# coding=gbk
# 本地摄像头推流
import cv2
import mediapipe as mp
import numpy as np
import subprocess as sp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic
mp_pose = mp.solutions.pose

# 调用ai算法进行帧处理并返回（这里使用Mediapipe骨骼点检测算法，读者可自行更改）
def frame_handler(image):
    with mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as holistic:
        image.flags.writeable = False
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = holistic.process(image_rgb)
        if results.pose_world_landmarks is not None:
            # 判断是否捕获到人体
            coords = np.array(results.pose_world_landmarks.landmark)
            # 汇总所有点的XYZ坐标
            def get_x(each):
                return each.x
            def get_y(each):
                return each.y
            def get_z(each):
                return each.z
            # 分别获取关键点XYZ坐标
            points_x = np.array(list(map(get_x, coords)))
            points_y = np.array(list(map(get_y, coords)))
            points_z = np.array(list(map(get_z, coords)))
            # 将三个方向坐标合并
            points = np.vstack((points_x, points_y, points_z)).T
            # 画图
            image.flags.writeable = True
            # 在关节点渲染
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
            image = cv2.flip(image, 1)
    return image

# 请读者自行修改url
# RTSP源地址src, 指定从哪里拉取视频流，测试时可以用VLC工具进行rtsp推流
src = "rtsp://:8554/"
# RTMP推流地址dst, 指定 用opencv把各种处理后的流(视频帧)推到哪里
dst = "rtmp://192.168.0.1:1935/live/test"
# 打开RTSP流，也可以用0，调用本地视频流，并取出视频流的帧率、帧宽、帧高
cap = cv2.VideoCapture(src)
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# ffmpeg command 保存进程参数
command = ['ffmpeg',
           '-y',
           'rawvideo',
           '-vcodec', 'rawvideo',
           '-pix_fmt', 'bgr24',
           '-s', "{}x{}".format(width, height),
           '-r', str(fps),
		   '-c:v', 'libx264',
           '-pix_fmt', 'yuv420p',
           '-preset', 'ultrafast',
           '-f', 'flv',
           '-g', '5',
           dst]

# 获取视频流的基本信息
fps = int(cap.get(cv2.CAP_PROP_FPS))
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))


# 建立子进程(配置管道)
pipe = sp.Popen(command, stdin=sp.PIPE)
# 循环读取视频流
while True:
    ret, frame = cap.read()  # 从视频流中获取一帧
    if not ret:
        raise IOError("could't open webcamera or video")
    # 处理代码(使用AI算法)
    frame_handler(frame)
    cv2.imshow('Video', frame)  # 显示处理结果

    # 推流代码
    # pipe.stdin.write(frame.tostring())

    # 按下q键退出
    if cv2.waitKey(1) == ord('q'):
        break

# 释放视频流
cap.release()
# 关闭窗口
cv2.destroyAllWindows()
# 关闭进程
pipe.stdin.close()
pipe.wait()
