# calibrate the camera by using a 10x8 grid (9x7 grid of corners) which is a black and white checkerboard, with each square
#    of the checkerboard being 1 inch (0.0245m) across
rosrun camera_calibration cameracalibrator.py camera:=/camera/ir image:=/camera/ir/image --size 9x7 --square 0.0245
