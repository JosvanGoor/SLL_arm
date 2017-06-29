


import ecto

import ecto
from ecto_opencv.highgui import imshow, ImageReader, VideoCapture, MatPrinter
from ecto_opencv.calib import PatternDetector, PatternDrawer,\
     FiducialPoseFinder, PoseDrawer, \
     CHESSBOARD, CameraIntrinsics, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv import highgui, calib, imgproc
import sys
import argparse

import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs


ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo


sub_rgb = ImageSub("image_sub",topic_name='/camera/rgb/image_color')
sub_depth = ImageSub("depth_sub",topic_name='/camera/depth_registered/image_raw')

sub_rgb_info = CameraInfoSub("image_info", topic_name='/camera/rgb/camera_info')

im2mat_rgb = ecto_ros.Image2Mat()
im2mat_depth = ecto_ros.Image2Mat()

im2info = ecto_ros.CameraInfo2Cv()

offset_x = -0.3095 #TODO: FIXME hard coded
offset_y = -0.1005
rows = 5
cols = 3
square_size = 0.1 #2.5 cm
pattern_type = ASYMMETRIC_CIRCLES_GRID

cd_bw = PatternDetector('Dot Detector, B/W',
                                                rows=rows, cols=cols,
                                                pattern_type=pattern_type,
                                                square_size=square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y)

offset_x = 0.1505
cd_wb = PatternDetector('Dot Detector, W/B',
                                                rows=rows, cols=cols,
                                                pattern_type=pattern_type,
                                                square_size=square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y)

circle_drawer = PatternDrawer(rows=rows, cols=cols)
circle_drawer2 = PatternDrawer(rows=rows, cols=cols)

gather = calib.GatherPoints("gather", N=2)
quantizer = imgproc.Quantize('Quantizer', alpha=1, beta=0)
rgb2gray = cvtColor(flag=Conversion.RGB2GRAY)
invert =imgproc.BitwiseNot()

poser = FiducialPoseFinder()
pose_drawer = PoseDrawer()
fps = highgui.FPSDrawer()

'''
graph = [
	sub_rgb["output"]>>im2mat_rgb["image"],
	im2mat_rgb['image'] >> (circle_drawer['input'],
	rgb2gray['image']),
	rgb2gray['image'] >> quantizer[:],
	rgb2gray['image'] >> cd_bw ['input'],
	cd_bw [ 'ideal', 'out', 'found'] >> poser['ideal', 'points', 'found'],
	sub_rgb_info["output"] >> im2info["camera_info"],
	im2info['K'] >> poser['K'],
	cd_bw ['out', 'found'] >> circle_drawer['points', 'found'],
	poser['R', 'T'] >> pose_drawer['R', 'T'],
	poser['R'] >> MatPrinter(name='R')['mat'],
	poser['T'] >> MatPrinter(name='T')['mat'],
	circle_drawer['out'] >> pose_drawer['image'],
	im2info['K'] >> pose_drawer['K'],
	pose_drawer['output'] >> imshow(name='Pose', waitKey=1)['image'],
	]


'''

graph = [
	sub_rgb["output"]>>im2mat_rgb["image"],
	im2mat_rgb['image'] >> rgb2gray['image'],
	rgb2gray['image'] >> quantizer[:],
	quantizer[:] >> (invert[:], cd_bw['input']),
	cd_bw['found', 'ideal', 'out'] >> gather['found_0000', 'ideal_0000', 'points_0000'],
	cd_wb['found', 'ideal', 'out'] >> gather['found_0001', 'ideal_0001', 'points_0001'],
	invert[:] >> cd_wb['input'], 
	gather['out', 'ideal', 'found'] >> poser['points', 'ideal', 'found'],
	sub_rgb_info["output"] >> im2info["camera_info"],
#	im2info['K'] >> poser['K'],
	im2mat_rgb['image'] >> circle_drawer['input'],
	circle_drawer['out'] >> circle_drawer2['input'],
	cd_bw['out', 'found'] >> circle_drawer['points', 'found'],
	cd_wb['out', 'found'] >> circle_drawer2['points', 'found'],
	circle_drawer2['out'] >> fps[:],
	poser['R', 'T'] >> pose_drawer['R', 'T'],
	circle_drawer2['out'] >> pose_drawer['image'],
	poser['R'] >> MatPrinter(name='R')['mat'],
	poser['T'] >> MatPrinter(name='T')['mat'],		
	pose_drawer['output'] >> imshow(name='pose', waitKey=1)['image'],
]

plasm = ecto.Plasm()
plasm.connect(graph)
ecto.view_plasm(plasm)

if __name__ == '__main__':
	ecto_ros.init(sys.argv,"image_node")
#	do_ecto()
	from ecto.opts import doit
	doit(plasm, description='Capture a video from the device and display it.')