#!/bin/bash
gnome-terminal -t "yolo_two" -x bash -c "python /home/wangzirui/HIT-DLR/src/Yolov5_ros/yolov5_ros/yolov5_ros/yolov5/detect_task_two.py;exec bash;"
sleep 10

gnome-terminal -t "server" -x bash -c "rosrun task2 server_two;exec bash;"
sleep 3

gnome-terminal -t "main" -x bash -c "rosrun task2 main_two;exec bash;"
sleep 3

gnome-terminal -t "right_hand_camera" -x bash -c "python /home/wangzirui/HIT-DLR/src/Yolov5_ros/yolov5_ros/yolov5_ros/yolov5/detect_box.py;exec bash;"
sleep 3

gnome-terminal -t "state_machine_two " -x bash -c "rosrun task2 state_machine_two;exec bash;"
sleep 30

gnome-terminal -t "left_hand_camera" -x bash -c "python /home/wangzirui/HIT-DLR/src/Yolov5_ros/yolov5_ros/yolov5_ros/yolov5/detect_cattle.py;exec bash;"
sleep 3