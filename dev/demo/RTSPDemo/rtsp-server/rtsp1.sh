#!usr//bin/bash
echo "start ~"
# start rtsp server
exec ./rtsp-simple-server &
# bash ~/Project/RTSPDemo/rtsp-server/rtsp1.sh
echo "server start ~"
sleep 1
#start push the stream
# bash ~/Project/RTSPDemo/rtsp.sh
# ffmpeg -re  -stream_loop -1 -i ./vid.mp4 -c:v libx265  -c:a copy   -preset:v ultrafast -tune:v zerolatency -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/mystream

# ffmpeg -re  -stream_loop -1 -i  /dev/video0 -c:v libx264  -c:a copy   -preset:v ultrafast -tune:v zerolatency -f rtsp -rtsp_transport tcp  rtsp://127.0.0.1:8554/mystream


# screen -d -m -S roscore bash -c 'roscore ; exec /bin/bash'
# while true; do
#   if [ "$(rosnode list)" ]; then
#     echo "has core"
#     break
#   fi
#   sleep 1
#   echo "wait core start ~"
# done
# echo "roscore success"
# #source ~/catkin_ws/devel/setup.bash
# #screen -d -m -S name bash -c 'roslaunch your_ros_pkg your_ros_pkg.launch; exec /bin/bash'

# while true; do sleep 1; done/Project/RTSPDemo/rtsp-server