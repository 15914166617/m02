tf查看
ros2 run tf2_tools view_frames
ros2 run rqt_tf_tree rqt_tf_tree

地图保存
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0

编译
colcon build --packages-select kaiaai_bringup
colcon build --packages-select makerspet_loki

键盘
ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_loki

//静态建图 
ros2 launch kaiaai_bringup b5.launch.py   robot_model:=makerspet_loki   configuration_basename:=backpack_2d.lua

//动态建图
ros2 launch kaiaai_bringup b6.launch.py robot_model:=makerspet_loki slam:=True
ros2 launch kaiaai_bringup b6.launch.py robot_model:=makerspet_loki map:=$HOME/map.yaml slam:=False

日记
1.0     结构重构完成
1.1     需要把必要的可设置参数拎出来


