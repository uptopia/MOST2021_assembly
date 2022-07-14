# MOST2021 Assembly

## Dependencies
**To use VISION only**
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* Install [cv_bridge for Python3]()

**To use lightweight_obj_detect** (optional)
* Requirements see [lightweight_obj_detect/README.md](https://github.com/ycxxn/lightweight_obj_detect/blob/main/README.md)

**To use TIMDA dual arm** (optional)
* Requirements see [timda_dual_arm/README.md](https://github.com/tku-iarc/timda_dual_arm/blob/6bcc5d341ad199a81be02582018ff8420166173e/README.md)

## Software environment, Platform
* Ubuntu 18.04, 20.04
* ROS Melodic, Noetic
* Python 3.6.9 (?)
* opencv 4.5.1 cv2 (?)
* cv_bridge (python3 待測試) (?)
* Install pcl (?)
* Install cv_bridge(?)

## Function Blocks
### 1. 物件偵測 (Object Detection) obj_detect
### 2. 部件分割 (Part Segmentation) part_afford_seg
### 3. 馬達姿態估測 (Motor Pose Estimation) motor_pose_est
### 4. 夾取姿態估測 (Grasp Pose Estimation) grasp_pose_est

<img src="readme_img/demo_graph.png" alt="drawing" width="300"/>  

rqt_graph 
<img src="readme_img/rosgraph_20220520_new.png" alt="drawing" width="300"/>  


## Installation and Setup
---
### Clone this repo and submodules
```
mkdir MOST2021_assembly
cd MOST2021_assembly
git clone --recursive https://github.com/uptopia/MOST2021_assembly.git src
```

**To use VISION only, without lightweight_obj_detect, timda_dual_arm**
* 在不需要編譯的資料夾(如lightweight_obj_detect, timda_dual_arm)中，創一個名為CATKIN_IGNORE的資料夾。

**To use lightweight_obj_detect** (optional)
* Requirements see [lightweight_obj_detect/README.md](https://github.com/ycxxn/lightweight_obj_detect/blob/main/README.md)

**To use TIMDA dual arm** (optional)
* Requirements see [timda_dual_arm/README.md](https://github.com/tku-iarc/timda_dual_arm/blob/6bcc5d341ad199a81be02582018ff8420166173e/README.md)


### Build
```
cd MOST2021_assembly
catkin_make
```

## Run
---
<terminal 1>
. devel/setup.bash
roslaunch get_motor get_motor.launch

<terminal 2>
. devel/setup.bash
rosrun get_motor get_motor

===========================
<terminal 1>
cd ~/realsense_ros
. devel/setup.bash
roslaunch realsense2_camera rs_rgbd.launch

或者播放預錄好的rosbag
roscore
cd ~/MOST2021_assembly
rosbag play -l MOST2021_assembly_rosbag.bag
rostopic list 

<terminal 2>
cd ~/MOST2021_assembly
. devel/setup.bash
rosrun select_workspace select_workspace.py

<terminal 3>
cd ~/MOST2021_assembly
. devel/setup.bash
rosrun obj_detect Det_Node.py

<terminal 4>
cd ~/MOST2021_assembly
. devel/setup.bash
rosrun part_afford_seg Afford_Node.py

<terminal 5>
cd ~/MOST2021_assembly
. devel/setup.bash
rosrun motor_pose motor_pose

<terminal 6>
cd ~/MOST2021_assembly
. devel/setup.bash
rosrun get_screw_cloud get_screw_cloud
```

### 執行手臂策略
Method 1. 雙臂策略狀態機
```
cd ~/dual_arm_ws
. devel/setup.bash
roslaunch manipulator_h_manager dual_arm.launch en_sim:=true

cd ~/dual_arm_ws
. devel/setup.bash
rosrun strategy assembly_grasp_strategy.py
```

Method 2. FlexBe 雙臂策略
