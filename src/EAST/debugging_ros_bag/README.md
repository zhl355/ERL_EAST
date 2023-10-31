# This package is used to replay ros bag and debugging. 


> private pkg
 

## Usage


* In the 1st terminal run 
  
    ```bash
    # start this ros bag debug 

    roslaunch debugging_ros_bag examine_ros_bag.launch

    ``` 


* In the 2nd terminal play rosbag
  
    ```bash
    # play ros bag
    rosbag play --clock -s 0 -r 1 [ROS_BAG_RECORDED].bag

    ``` 

## Refs
* learn ros bag usage at [here](https://wiki.ros.org/rosbag)