#这是一个rtmp 推流 python3.8 demo can be used but fps decreases 
import cv2
 
# subprocess 模块允许我们启动一个新进程，并连接到它们的输入/输出/错误管道，从而获取返回值。
import subprocess
 
# 视频读取对象
cap = cv2.VideoCapture(0)
 
# 推流地址
rtmp = "rtmp://127.0.0.1:1935/live/example"# 推流的服务器地址
# rtmp = "rtsp://127.0.0.1:8554/live/test"
 
# 设置推流的参数
command = ['ffmpeg',
           '-y',
           '-f', 'rawvideo',
           '-vcodec', 'rawvideo',
           '-pix_fmt', 'bgr24',
           '-s', '640*480',  # 根据输入视频尺寸填写
           '-r', '30',
           '-i', '-',
           '-c:v', 'h264',
           '-pix_fmt', 'yuv420p',
           '-preset', 'medium',
           '-f', 'flv',
           rtmp]
 
 
# 创建、管理子进程
pipe = subprocess.Popen(command, stdin=subprocess.PIPE)
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
 
# 循环读取
while cap.isOpened():
    # 读取一帧
    ret, frame = cap.read()
    if frame is None:
        print('read frame err!')
        continue
 
    # 显示一帧
    # fps = int(cap.get(cv2.CAP_PROP_FPS))
    cv2.imshow("frame", frame)
 
    # 按键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
    # 读取尺寸、推流
    _, send_data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])
    img = cv2.resize(frame, size)
    # 压缩图片
    
    pipe.stdin.write(send_data.tobytes())
    
 
# 关闭窗口
cv2.destroyAllWindows()
 
# 停止读取
cap.release()