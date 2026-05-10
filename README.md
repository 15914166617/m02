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
1.2     问题：后续可以优化，系统保活如果重启，他只是启动了下位机，上位机要重启吗？后续再考虑，目前基础包活已经可以了。（好像没这个问题？？）
1.3     电机相关的引脚，和pid参数可一在外部修改了
1.4     数据融合模块，注释代码的精简完成
1.5     雷达模块 串口发送设置-1禁用，pwm确定无效引脚
1.6     上电时和，程序下载完毕时，小车会当次右转一次，或者说按住复位键时，电机会一直转，这个是为什么？
