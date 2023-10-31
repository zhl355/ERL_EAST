# Stop Now

This package implement functionality that using PS remote console controller 
to act as emergency stop button. 


### Advanced Version of Emergency Stop Package for twist_mux

1. Add program emergency stop trigger.
2. Configurable Joystick buttons from launch file.
3. Ready to run as an individual node for `ref_gvn_ros`.

### Config Table
1. `use_human_interrupt` : default `True`, once set to false, `use_program_interrupt` must be set to true and correctly configured, otherwise it will be automatically set to true.
2. `use_program_interrupt` : default `False`, whether to use program interruption. If configured to be true, then the following configurations are needed to be provided, otherwise it will be automatically set to false. Please see `erl_msgs/EmergencyStopMsg.msg` for detailed topic configuration.
- `monitoring_topic` : rostopic used to trigger emergency stop;
3. `hold_to_run` : default `False`, whether human operator needs to hold certain button on joystick to enable motor on robot.
4. `joy_axes` : default `5`, this config specify which axis of joystick we use to serve as emergency stop button, if this term is specified, then it will use the assigned axes as emergency stop button NO MATTER whether joy_buttons is provided or not, PS4 trigger R2 is joy_axes 5, primary config using R2, you can set alternative axes.
5. `joy_buttons` : default `3`, this config specify which button on joystick we use on joystick to serve as emergency stop button, ONLY VALID if joy_axes is not provided, PS4 button square is joy_buttons 3, primary config using square, you can set alternative buttons.

### Advanced Features
1. `use_custom_topic`: whether to use topic type other than `erl_msgs/EmergencyStopMsg.msg`. If configured to be true, then in addition to `monitoring_topic`, the following parameters should be provided.
- `monitoring_topic_msg_module` : python module contains the corresponding rostopic message definition, for import use;
- `monitoring_topic_msg_type` : the exact rostopic message type, for import use, for example, if one want to import `geometry_msgs/Pose2D`, then the `monitoring_topic_msg_module` term should be set to `"geometry_msgs.msg._Pose2D"` and the `monitoring_topic_msg_type` term should be set to `"Pose2D"`;
- `program_interrupt_variable` : variable contains in the message provided above used to trigger emergency stop, support Bool and Int, if the variable type is Int, then the following configuration is needed to be provided:
 - `program_interrupt_threshold` : if the variable type is Int, then less than this value will trigger emergency stop.
