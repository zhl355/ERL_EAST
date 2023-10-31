# Environment Aware and Safe Tracking (EAST)

This repository contains code for the EAST project as supplementary material for 
the paper submission at IJRR. ArXiv version is now available at [here](https://arxiv.org/pdf/2310.01363.pdf).

This code demonstrate the capability of EAST algorithm in various environments for safe and environemnt aware
navigation in challenging environments. 

This code is primarily developed by [Zhichao Li](https://zhl355.github.io/) , [Yinzhuang Yi](https://scholar.google.com/citations?user=ehBs-OYAAAAJ&hl=en) and [Zhuolin Niu](https://www.linkedin.com/in/zhuolin-n-6b4417225/) from [Exsitential Robototics Laboratory](http://erl.ucsd.edu/) led by Professor [Nikolay Atanasov](https://natanaso.github.io/) at UCSD.

## Dependencies

The code is tested on `Ubuntu 20.04 LTS + ROS Noetic` 

#### ROS libaries
  + JACKAL `sudo apt install ros-noetic-jackal-*`
  + HECTOR_MAPPING `sudo apt install ros-noetic-hector-mapping`
  + RQT_MULTIPLOT `sudo apt install ros-noetic-rqt-multiplot`
  + PYBIND11_CATKIN `sudo apt install ros-noetic-pybind11-catkin`

#### Non-ROS libaries
  + Eigen3  `sudo apt install -y libeigen3-dev`
  + Boost `sudo apt install-y libboost-dev`
  + OPENCV `sudo apt install libopencv-dev python3-opencv`
  + CVXPY `pip install cvxpy` with your favourite QCQP solver (MOSEK, Gurobi, etc.)
  + CGAL `sudo apt install libcgal-dev`

## Complication

Download this repo, create working space and use `catkin build` tool to build it just as other 
ROS packages. Don't forget to source the environment afterward. All non-official packages have been included. 

**Be patient, Gazebo starts slow for initial installation. Wait until the robot shows up in Rviz or enable Gazebo GUI in launch file.**
## Demonstration 


#### Evaluate EAST in simulation at **Jackal Race World**. 

Run the following script in ros-sourced terminal first,
```sh
roslaunch ref_gvn_ros ref_gvn_cone.launch
```

and click goal point within map using RViz `Nav2D` button. 

<br>

#### Evaluate EAST in simulation at **U-shape** to see the effect of SDDM boost. By running  

Run the following script in ros-sourced terminal first,

  + sddm boost enabled 
      ```sh
      roslaunch ref_gvn_ros ushape_sddm_on.launch
      ```
  
  + sddm boost disabled 
      ```sh
      roslaunch ref_gvn_ros ushape_sddm_off.launch
      ```
  
  and click any point on the map using  RViz `Nav2D` button. 

<br>

#### Evaluate EAST in simulation at **Maze**.
Run the following script in ros-sourced terminal first,

```sh
roslaunch ref_gvn_ros ref_gvn_cone_maze.launch
```
  
  and click goal at (6, 6) using RViz `Nav2D` button on the map. 
  This experiment might experience de-localization problem randomly. Feel free to test our algorithm in other maps/worlds.


#### Evaluate EAST in simulation at **Jackal Race World** with virtual moving obstacles.
Run the following script in ros-sourced terminal first,

```sh
roslaunch ref_gvn_ros act_refgvn_virtual_movobs.launch
```
  
and click goal point within map using RViz `Nav2D` button. 

Note that, there is no gurantee for all time safety, instead, our controller just tries its best to avoid moving obstacle.
No action will be taken when goal is reached. 


## Citing this work

If you find the ideas and code implementation are useful for your own work, 
please check out the following paper and more related projects at my [website](https://zhl355.github.io/).


* Environment Aware and Safe Tracking (EAST) [[pdf]](https://arxiv.org/pdf/2310.01363.pdf)

  ```txt
    @inproceedings{li_east,
      author = {Z. Li, Y. Yi, Z. Niu and N. Atanasov},
      title = {EAST: Environment Aware Safe Tracking using Planning and Control Co-Design},
      journal = {arXiv preprint arXiv:2106.13176},
      year = {2023}
    }
  ```
  
