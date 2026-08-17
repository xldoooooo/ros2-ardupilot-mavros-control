from socket import *
import cv2
import numpy as np
 
# sk = socket()
sk = socket(AF_INET, SOCK_STREAM)
# sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 确保下次启动是端口保留
# sk.bind(('192.168.1.112', 8081))  # 这里也是写本服务机的ip，端口随便写
sk.bind(('127.0.0.1', 8081))  # 这里也是写本服务机的ip，端口随便写


sk.listen(5)
conn, address = sk.accept()
 
while True:
    try:
        data = conn.recv(88888)  # 88888为接受的最大字节数（默认分辨率情况下图片也就3、4万字节）
        nparr = np.fromstring(data, dtype='uint8', sep='')  # 化为数组
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)  # 解码为彩色图
        cv2.imshow('dabo', img)
        if cv2.waitKey(1) >= 0:  # 每1秒呈现一帧图片，按键盘任何键结束
            break
    except Exception as e:  # 打印出特定错误（个人习惯）
        print(e)
        
conn.close()
sk.close()