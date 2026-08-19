import cv2
import subprocess
 
# 视频读取对象
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
# cap = cv2.VideoCapture("/home/zzx/vid.mp4")

# 推流地址
rtsp = "rtsp://127.0.0.1:8554/live/test1"# 推流的服务器地址
# Stream #0:0: Video: hevc (libx265), yuv422p, 640x480, q=2-31, 30 fps, 90k tbn, 30 tbc
# 设置推流的参数

fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("fps=========== ",fps, width, height)
command = ['ffmpeg',
           '-y',
           '-f', 'rawvideo',
           '-vcodec', 'rawvideo',
           '-pix_fmt', 'bgr24',
           '-s', "{}x{}".format(width, height),  # 根据输入视频尺寸填写
           '-r',  str(fps),
           '-i', '-',
           '-c:v', 'libx265',
           '-pix_fmt', 'yuv422p',
           '-preset', 'medium',
           '-f', 'rtsp',
           rtsp]

# 创建、管理子进程
pipe = subprocess.Popen(command,shell=False, stdin=subprocess.PIPE)
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

# 循环读取
num = 0
while cap.isOpened():
    # num = num + 1
    # print(num)
    # 读取一帧
    ret, frame = cap.read()
    if frame is None:
        print('read frame err!')
        continue
 
    # 显示一帧
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    cv2.imshow("frame", frame)
    # print("fps:",fps)
 
    # 按键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # _, img_encode = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
    # 读取尺寸、推流
    img_encode = cv2.resize(frame, size)
 
    pipe.stdin.write(img_encode.tobytes())

# 关闭窗口
cv2.destroyAllWindows()
 
# 停止读取
cap.release()




# import subprocess as sp
# import cv2 as cv

# rtmpUrl = "rtsp://127.0.0.1:8554"
# camera_path = ""
# cap = cv.VideoCapture(0)

# # Get video information
# fps = int(cap.get(cv.CAP_PROP_FPS))
# width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
# height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

# # ffmpeg command
# command = ['ffmpeg',
#         '-y',
#         '-f', 'rawvideo',
#         '-vcodec','rawvideo',
#         '-pix_fmt', 'bgr24',
#         '-s', '640*480',#"{}x{}".format(width, height),
#         '-r', '30',#str(fps),
#         '-i', '-',
#         '-c:v', 'libx264',
#         '-pix_fmt', 'yuv420p',
#         '-preset', 'ultrafast',
#         '-f', 'flv', 
#         rtmpUrl]

# # 管道配置
# p = sp.Popen(command, shell=False, stdin=sp.PIPE)
# size = (int(cap.get(cv.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv.CAP_PROP_FRAME_HEIGHT)))

# # read webcamera
# while(cap.isOpened()):
#     ret, frame = cap.read()
#     if not ret:
#         print("Opening camera is failed")
#         break

#     # process frame
#     # your code
#     # process frame

#     # write to pipe
#     # 读取尺寸、推流
#     img = cv.resize(frame, size)
#     p.stdin.write(frame.tostring())

# # 关闭窗口
# cv.destroyAllWindows()
 
# # 停止读取
# cap.release()









# import cv2
# import ffmpeg

# # RTSP服务器地址
# # rtsp_server = "rtsp: //your_username: your_password@your_rtsp_server:port/live.sdp"
# rtsp_server = "rtsp://test:test@127.0.0.1:8554/live.sdp"
# # 捕获视频流
# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("Error: Could not open video device.")
#     exit()


# # 设置视频参数
# frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# # 使用FFmpeg推流
# process = (
#     ffmpeg
#     .input('pipe:',format='rawvideo', pix_fmt='bgr24', s='{}x{}'.format(frame_width, frame_height))
#     .output(rtsp_server, codec='libx264', preset='ultrafast', framerate=30)
#     .overwrite_output()
#     .run_async (pipe_stdin=True)
# )

# while True:
#     # 读取一帧
#     ret, frame = cap.read()
#     if not ret:
#         print("Error: Could not read frame.")
#         break

#     # 将帧写入FFmpeg的stdin
#     process.stdin.write(frame.tobytes())

# # 释放资源

# cap.release()
# process.stdin.close()
# process.wait()