# RobotCalibration
Find coordinate systems transformation between RGBD camera axes and robot's ones.

![Alt text](readme_files/capture1.png?raw=true "Title")

Compute the transformation based on 4+ positions of any robot with pictures of robot in every position.
To find the positions of a robot's manipulator from a camera, the robot should hold a color ball.

## Usage

Install requirements.txt

1. Put a sequence of calibration images and validation images of a robot in the `images` and 'validation' folders
2. Save the corresponding coordinates of a manipulator in the robot coordinate system to the `positions/positions.json`
3. Set up a RoI in `configs/roi.json` and a matrix of intrinsic camera parameters in  `configs/camera_matrix.json`
4. Run `python3 calibrate.py` or use jupyter notebook `calibrate.ipynb`
5. Result transformation is saved in 'transformation.npy', also its accuracy on the validation set will be printed in calibrate.py

## Dependencies
* python3
* opencv (3.3.1)
* numpy (1.14.2)

