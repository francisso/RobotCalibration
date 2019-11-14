import cv2
import numpy as np
import json
import os
import argparse

parser = argparse.ArgumentParser(description='Compute translation between RGBD camera coordinates system and a robot\'s')
parser.add_argument('--images', metavar='images', default='images/', 
                    help='path to captured images')
parser.add_argument('--positions', metavar='positions', default='positions/positions.txt',
                    help='path to robot positions')
parser.add_argument('--br', '--ball-radius', default=24, type=float,
                    help='ball radius', dest='ball_radius')

def get_center(image, mask):
    """Return center of the largest red ball
    
    Keyword arguments:
    image -- numpy array image in BGR color space
    mask -- numpy array with region of interest. Should be the same shape as image. Use (255,255,255) for points in RoI and (0,0,0) for points outside
    """
    im = cv2.GaussianBlur(image, (7,7), 0)
    im = cv2.bitwise_and(mask, im)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    saturation = hsv[...,1]
    saturation[(hsv[...,0] > 15) & (hsv[...,0] < 165)]=0
    _, im1 = cv2.threshold(saturation, 92, 255, cv2.THRESH_BINARY)
    contours = cv2.findContours(im1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
    contour = max(contours, key=cv2.contourArea)
    b_rect = cv2.boundingRect(contour)
    return (b_rect[0]+b_rect[2]/2, b_rect[1]+b_rect[3]/2)

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

def main():
    args = parser.parse_args()
    number_images = len(os.listdir(args.images))//2
    print(f"Total number of RGBD images {number_images}")
    if number_images < 1:
        print("No images found")
        return 
    #Load 1 images
    example = cv2.imread(os.path.join(args.images, "color_0.png"))
    #Create ROI mask
    with open("configs/roi.json") as f:
        roi = json.load(f)
    print(roi)
    mask = np.zeros_like(example)
    mask = cv2.rectangle(mask, (roi['x'], roi["y"]), (roi['x']+roi["width"], roi["y"]+roi["height"]), (255,255,255), -1)
    #Compute camera coordinates
    with open("configs/camera_matrix.json") as f:
        camera_matrix = json.load(f)
    camera_coords = [get_world_coords_n(i, mask, camera_matrix, args.images) for i in range(number_images)]
    for i in range(number_images):
        d = np.linalg.norm(camera_coords[i])
        camera_coords[i] = camera_coords[i]*(d+args.ball_radius)/d
    #Load robot coordinates
    with open("positions/positions.txt") as f:
        posiitons = json.load(f)["positions"]
    robot_coords = np.zeros((len(posiitons),3))
    for i in range(len(posiitons)):
        robot_coords[i,0] = posiitons[i]["x"]
        robot_coords[i,1] = posiitons[i]["y"]
        robot_coords[i,2] = posiitons[i]["z"] - args.ball_radius
    camera_coords = np.array(camera_coords).astype(np.float32)
    robot_coords = np.array(robot_coords).astype(np.float32)
    #compute calibration
    trans = cv2.estimateAffine3D(camera_coords, robot_coords)[1]
    print(trans)
    print("determinant is: ", np.linalg.det(trans[:,:3]))
    


if __name__ == '__main__':
    main()





