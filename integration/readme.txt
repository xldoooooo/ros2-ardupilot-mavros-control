这是这次我们用的websocket协议Java服务器端（只是消息通道，没有业务逻辑），我测试通过了的，你试试。@孙长卿 



部署简单，安装如下版本的jdk(不低于此版本吧)后，在linux端执行java -jar websocket-server-1.0.0.jar & 就行。
java version "1.8.0_121"
Java(TM) SE Runtime Environment (build 1.8.0_121-b13)
Java HotSpot(TM) 64-Bit Server VM (build 25.121-b13, mixed mode)



服务器URL是：ws://192.168.xx.xx:8581/ws



注意，生产环境的URL不是这样。



![991c824229cadf9e3ab283c459e62ba5](/home/nvidia/scq/projects/ros2-ardupilot-sitl-hardware/integration/assets/991c824229cadf9e3ab283c459e62ba5.png)