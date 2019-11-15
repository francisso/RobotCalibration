import cv2
import numpy as np
import json
import os
import argparse
import glob

parser = argparse.ArgumentParser(description='Compute translation between RGBD camera coordinates system and a robot\'s')
parser.add_argument('--images', metavar='images', default='images/', 
                    help='path to captured images')
parser.add_argument('--validation', metavar='validation', default='validation/', 
                    help='path to validation images')
parser.add_argument('--positions', metavar='positions', default='positions/positions.json',
                    help='path to robot positions')
parser.add_argument('--roi', metavar='roi', default='configs/roi.json',
                    help='path to robot positions')
parser.add_argument('--cam', metavar='camera_matrix', default="configs/camera_matrix.json",
                    help='camera matrix')
parser.add_argument('--br', '--ball-radius', default=24, type=float,
                    help='ball radius', dest='ball_radius')

def get_center(image, mask):
    """Return center of the largest red ball
    
    Keyword arguments:
    image -- numpy array image in BGR color space
    mask -- numpy array with region of interest. Should be the same shape as image. Use (255,255,255) for points in RoI and (0,0,0) for points outside
    """
    im = cv2.GaussianBlur(image, (7,7), 0)*mask
    im=im.astype(np.uint8)

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    saturation = hsv[...,1]
    saturation[(hsv[...,0] > 15) & (hsv[...,0] < 165)]=0
    _, im1 = cv2.threshold(saturation, 92, 255, cv2.THRESH_BINARY)

    _, contours, _ = cv2.findContours(im1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    contour = max(contours, key=cv2.contourArea)
    b_circle = cv2.minEnclosingCircle(contour)

    return b_circle[0]

def get_world_coords(x,y, depth, camera_matrix):
    """return physical coordinates in mm
    
    Keyword arguments:
    x, y -- coordinates of a point in pixels
    depth -- depth coordiante of the same point
    camera_matrix -- 3x3 matrix with focal lengthes and principial point"""
    f = np.linalg.inv(camera_matrix)
    v = np.array([x,y,1]) * depth
    return np.dot(f,v)

def get_world_coords_n(n, mask, camera_matrix, base_path="images/"):
    """ return xyz coordinates of the largest red ball for an image and depth number n
    
    Keyword arguments:
    n -- number of image in the folder
    base_path -- path to the folder, conatining calibration images (default: images)
    """
    i0 = cv2.imread(os.path.join(base_path, "color_{}.png".format(n)))
    i1 = cv2.imread(os.path.join(base_path, "depth_{}.png".format(n)), cv2.IMREAD_UNCHANGED)
    center = get_center(i0, mask)
    depth = i1[int(center[1]), int(center[0])]
    return get_world_coords(center[0], center[1], depth, camera_matrix)

#main estimation function
def estimate_transformation(width,height,points,image_folder,camera_matrix,roi,ball_radius):
    '''return transformation matrix

    arguments:
       width,height - for images
       points - real points in robot axes
       image_foler - folder with image_{}.png depth_{}.png
       camera_matrix
       roi - rect of interest
       ball_radius - in mm

    note, that z is the vertical axis, so to calibrate we need to 
    '''

    number_images = len(glob.glob(image_folder+'*.png'))//2
    mask = np.zeros((height,width,3))
    mask[roi['y']:roi['y']+roi['height'],roi['x']:roi['x']+roi['width'],:]=1.0

    
    camera_coords = [get_world_coords_n(i, mask, camera_matrix, image_folder) for i in range(number_images)]
    for i in range(number_images):
        d = np.linalg.norm(camera_coords[i])
        print(camera_coords[i])
        camera_coords[i] += ball_radius/d
        print(camera_coords[i])
    
    sphere_center_in_robot_axes = np.zeros((len(points),3))
    for i in range(len(points)):
        sphere_center_in_robot_axes[i,0] = points[i]["x"]
        sphere_center_in_robot_axes[i,1] = points[i]["y"]
        sphere_center_in_robot_axes[i,2] = points[i]["z"] - ball_radius #robot grabs a ball in the upper point


    camera_coords = np.array(camera_coords).astype(np.float32)
    sphere_center_in_robot_axes = np.array(sphere_center_in_robot_axes).astype(np.float32)
    #compute calibration
    trans = cv2.estimateAffine3D(camera_coords, sphere_center_in_robot_axes)[1]

    return trans

def check_accuracy(transformation,width,height,points,image_folder,camera_matrix,roi,ball_radius):
    '''return average accuracy

    arguments:
       width,height - for images
       points - real points in robot axes
       image_foler - folder with image_{}.png depth_{}.png
       camera_matrix
       roi - rect of interest
       ball_radius - in mm

    note, that z is the vertical axis, so to calibrate we need to 
    '''

    av_acc=0

    number_images = len(glob.glob(image_folder+'*.png'))//2
    mask = np.zeros((height,width,3))
    mask[roi['y']:roi['y']+roi['height'],roi['x']:roi['x']+roi['width'],:]=1.0

    
    camera_coords = [get_world_coords_n(i, mask, camera_matrix, image_folder) for i in range(number_images)]
    for i in range(number_images):
        d = np.linalg.norm(camera_coords[i])
        print(camera_coords[i])
        camera_coords[i] += ball_radius/d
        print(camera_coords[i])
    print(points)
    sphere_center_in_robot_axes = np.zeros((len(points),3))
    for i in range(len(points)):
        sphere_center_in_robot_axes[i,0] = points[i]["x"]
        sphere_center_in_robot_axes[i,1] = points[i]["y"]
        sphere_center_in_robot_axes[i,2] = points[i]["z"] - ball_radius #robot grabs a ball in the upper point

    for i in range(len(camera_coords)):
        p=np.zeros((4))
        p[0:3]=camera_coords[i][0:3]
        p[3]=1.0
        p2=np.dot(transformation,p)
        av_acc+=np.linalg.norm(p2-sphere_center_in_robot_axes[i])

    
    return av_acc/len(camera_coords)

def main():
    args = parser.parse_args()

    number_images = len(glob.glob(args.images+'*.png'))//2
    print("Total number of RGBD images: ",number_images)

    if number_images < 1:
        print("No images found")
        return 
    #Load 1 images to get width,height
    example = cv2.imread(os.path.join(args.images, "color_0.png"))
    width=example.shape[1]
    height=example.shape[0]
    #ROI mask
    with open(args.roi) as f:
        roi = json.load(f)
    #Load robot poisitions
    with open(args.positions) as f:
        ps=json.load(f)
        points =ps ["positions"]
        val_points=ps["validation"]
    #Compute camera coordinates
    with open(args.cam) as f:
        camera_matrix = json.load(f)

    #estimate transformation
    trans=estimate_transformation(width,height,points,args.images,camera_matrix,roi,args.ball_radius)
    print(trans)
    print("determinant is: ", np.linalg.det(trans[:,:3]))

    average_accuracy=check_accuracy(trans,width,height,val_points,args.validation,camera_matrix,roi,args.ball_radius)
    print('av error: ' + str(average_accuracy))


    
if __name__ == '__main__':
    main()





