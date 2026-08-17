import socket
import cv2
import numpy as np

ip_port = ('127.0.0.1', 8081) # 这里填服务端的ip，端口要和服务端一致
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建套接字
s.connect(ip_port)  # 连接服务器
 
cap = cv2.VideoCapture(0)  # 打开摄像头，0为默认摄像头
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 352)  # 设置每帧图片的宽
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)  # 设置每帧图片的高
 
while True:
    ret, frame = cap.read()  # 获取视频的开启状态和每一帧图片
    # frame=cv2.resize(frame,(1280,720),interpolation=cv2.INTER_CUBIC)
    # img_encode = cv2.imencode('.jpg', frame)[1]  # 对每一帧图片进行编码
    img_encode = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 30])[1]  # 对每一帧图片进行编码
    data = np.array(img_encode)  # 转化为矩阵
    byte_encode = data.tobytes()  # 编码格式转为字节格式
    data_len = str(len(byte_encode))  # 获取每一帧图片的大小（字节数）
    print('每帧图片大小: %s' % data_len)
    s.send(byte_encode)  # 发送给客户端呈现