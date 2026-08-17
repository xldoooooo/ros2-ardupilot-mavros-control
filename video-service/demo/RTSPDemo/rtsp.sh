#!/bin/sh　　


ffmpeg -re  -stream_loop -1  -i /dev/video0   -c:v libx264  -c:a copy   -preset:v ultrafast -tune:v zerolatency -f rtsp -rtsp_transport tcp  rtsp://127.0.0.1:8554/mystream
