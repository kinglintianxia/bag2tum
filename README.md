# bag2tum
Parse ROS bag to TUM dataset style files.

## Steps
* First step

Choose the folder to save your dataset, say "~/image". You need to create `rgb` and `depth` folders to store images. It looks like this:
```shell
image
├── depth
└── rgb
```
* Second step

Change the `save_folder`,`rgb_topic` and `depth_topic` parameters to yours in bag2tum.launch file.
```shell
roslaunch bag2tum bag2tum.launch
```
* Third step

Play bag file
```shell
rosbag play *.bag
```
* At last, you will get rgb images, depth images, rgb.txt and depth.txt.
