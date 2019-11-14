# RobotCalibration
Find coordinate systems transformation between a RGBD camera and a robot's one.

![Alt text](readme_files/capture1.png?raw=true "Title")

Compute the transformation based on 4+ positions of any robot, with pictures of robot in every position.
To find the positions of a robot's manipulator from a camera, the robot should hold a color ball.

## Usage

1. Put images of robot in the `images` folder
2. Save coordinates in the robot coordinate system to the `positions/positions.txt`
3. Set up roi in `configs/roi.json` and camera matrix in  `configs/camera_matrix.json`
4. run `python3 calibrate.py` or use jupyter notebook `calibrate.ipynb`

## Dependencies
* python3
* opencv (4.1.0)
* numpy (1.14.2)

