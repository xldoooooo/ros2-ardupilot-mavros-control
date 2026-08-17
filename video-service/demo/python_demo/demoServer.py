'''
Author: 柒上夏OPO
Date: 2022-01-10 12:31:24
LastEditTime: 2022-01-14 14:52:54
LastEditors: CloudSir
Description: 
'''

import numpy as np
import cv2
from socket import *
import time

s = socket(AF_INET, SOCK_DGRAM) # 创建UDP套接字
# s = socket(AF_INET, SOCK_STREAM) # 创建TCP套接字
addr = ('127.0.0.1', 8081)  # 0.0.0.0表示本机
s.bind(addr)

s.setblocking(0) # 设置为非阻塞模式

# width = int(r_img.get(cv2.CAP_PROP_FRAME_WIDTH))
# height = int(r_img.get(cv2.CAP_PROP_FRAME_HEIGHT))
# print("width =%d, height = %d",width,height)
cv2.namedWindow('server',cv2.WINDOW_NORMAL)

# 创建 VideoWriter 对象，用于保存灰度视频
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 视频编码格式
# out = cv2.VideoWriter("/home/zzx/vidoutput.mp4", fourcc, 30, (640, 480), isColor=True)
t_pre = 0
while True:
    data = None
    try:
        data, _ = s.recvfrom(6220800) #921600
        receive_data = np.frombuffer(data, dtype='uint8')
        r_img = cv2.imdecode(receive_data, 1)

        cv2.putText(r_img, "server", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow('server', r_img)
        # out.write(r_img)
        size = r_img.shape
        print("width , height ",size[1],size[0])   
        t_now = int(round(time.time() * 1000))
        t_diff = t_now - t_pre
        print("fps = ", int(1000/t_diff))
        t_pre = t_now

    except BlockingIOError as e:
        pass

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

