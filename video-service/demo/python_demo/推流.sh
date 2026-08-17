# 启动推流服务器
~/Project/RTSPDemo/rtsp-server$ ./rtsp-simple-server


ffmpeg -re  -stream_loop -1  -i /dev/video0   -c:v libx264  -c:a copy   -preset:v ultrafast -tune:v zerolatency -f rtsp -rtsp_transport tcp  rtsp://127.0.0.1:8554/mystream

ffmpeg -re  -stream_loop -1 -i /dev/video0  -c:v libx265  -c:a copy   -preset:v ultrafast -tune:v zerolatency -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/mystream

# ffmpeg -re -stream_loop -1 -i /dev/video0 -vcodec h264 -acodec aac -f rtsp rtsp://192.168.16.230/live/test
# ffmpeg -re -stream_loop -1 -i ./clock.mp4 -vcodec h264 -acodec aac -f rtsp -rtsp_transport tcp rtsp://192.168.16.230/live/test

ffmpeg -i rtsp://127.0.0.1:8554/mystream -c copy outputrstp.mp4 -f segment -segment_time 10 stream_piece_%d.mp4
ffmpeg -i rtsp://127.0.0.1:8554/mystream -c copy -f segment -segment_time 60 stream_piece_%d.mp4

ffmpeg -f dshow -i /dev/video0 -vcodec libx264 -preset:v ultrafast -tune:v zerolatency -rtsp_transport udp -f rtsp rtsp://127.0.0.1/stream


grain(需要保留大量的grain时用)  
stillimage(静态图像编码时使用)
psnr(为提高psnr做了优化的参数)
ssim(为提高ssim做了优化的参数)
fastdecode(可以快速解码的参数)
zerolatency(零延迟，用在需要非常低的延迟的情况下，比如电视电话会议的编码)

 ultrafast superfast veryfast faster fast medium slow slower veryslow placebo

 https://blog.csdn.net/q1457797371/article/details/161458449