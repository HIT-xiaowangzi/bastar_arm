#!/bin/bash
gnome-terminal -t "wrist_camera" -x bash -c "python /home/wangzirui/ros_ws/task3.py;exec bash;"
sleep 1

gnome-terminal -t "yolo_three" -x bash -c "python /home/wangzirui/HIT-DLR/src/Yolov5_ros/yolov5_ros/yolov5_ros/yolov5/detect_task_three.py;exec bash;"
sleep 2

gnome-terminal -t "server" -x bash -c "rosrun task3 server_three;exec bash;"
sleep 1

gnome-terminal -t "main" -x bash -c "rosrun task3 main_three;exec bash;"
sleep 1

gnome-terminal -t "right_hand_camera" -x bash -c "python /home/wangzirui/HIT-DLR/src/Yolov5_ros/yolov5_ros/yolov5_ros/yolov5/detect_box.py;exec bash;"
sleep 2

gnome-terminal -t "left_hand_camera" -x bash -c "python /home/wangzirui/HIT-DLR/src/Yolov5_ros/yolov5_ros/yolov5_ros/yolov5/detect_ruler.py;exec bash;"

sleep 2
gnome-terminal -t "state_machine_three " -x bash -c "rosrun task3 state_machine_three;exec bash;"


