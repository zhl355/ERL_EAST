# jackal_control package

This package includes some config changes from the clearpath jackal_control package: https://github.com/jackal/jackal/tree/noetic-devel/jackal_control

* control.yaml 
  * Input noise removed.
  * Input bound changed. 

* robot_localization.yaml
  * change a bit following instruction at https://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html

* teleop_ps4.yaml [line 6-7]
    ```yaml
    axis_angular: 3    # default 0 (3 is right knob left/right axis zhl)
    scale_angular: 1.4    # range is [-1, 1], this scale caps angular vel [-1.4, 1.4]
    ```

* twist_mux.yaml
  * support keyboard [need additional package `teleop_twist_keyboard`]

