# 终端集成与日志修改



核心任务：

1. 删除当前GUI右上角的 终端 按钮 和左上角 文件 按钮里的开启终端功能

2. 终端集成到日志输出栏，类似vscode integrated terminal ; 用户可以通过点击按钮切换到日志界面或终端界面，并且支持关闭或新建终端。终端应该是跟linux的terminal一样的逻辑，也能sudo什么的

3. 当前日志，info级别消息太多，debug级别消息太少。修改适配：主要是mavros启动等的消息，要把等级从info降为debug。比如：像 “**[INFO ]** [sitl] Embedding file default_params/quadplane-tilttri.parm:Tools/autotest/default_params/quadplane-tilttri.parm

   11:04:35.677 **[INFO ]** [sitl] Embedding file default_params/quadplane-tilttrivec.parm:Tools/autotest/default_params/quadplane-tilttrivec.parm

   11:04:35.677 **[INFO ]** [sitl] Embedding file default_params/quadplane-tri.parm:Tools/autotest/default_params/quadplane-tri.parm

   11:04:35.677 **[INFO ]** [sitl] Embedding file default_params/quadplane.parm:Tools/autotest/default_params/quadplane.parm

   11:04:35.677 **[INFO ]** [sitl] Embedding file default_params/rover-omni3mecanum.parm:Tools/autotest/default_params/rover-omni3mecanum.parm

   11:04:35.677 **[INFO ]** [sitl] Embedding file default_params/rover-skid.parm:Tools/autotest/default_params/rover-skid.parm

   11:04:35.677 **[INFO ]** [sitl] Embedding file default_params/rover-vectored.parm:Tools/autotest/default_params/rover-vectored.parm

   11:04:35.677 **[INFO ]** [sitl] Embedding file default_params/rover.parm:Tools/autotest/default_params/rover.parm

   11:04:37.214 **[INFO ]** [sitl] Embedding file default_params/sailboat-motor.parm:Tools/autotest/default_params/sailboat-motor.parm

   11:04:37.214 **[INFO ]** [sitl] Embedding file default_params/sailboat.parm:Tools/autotest/default_params/sailboat.parm”

   这种的，应该是info

4. 现在日志模块右上角的“自动滚动”按钮，其功能存在bug, 点击之后，日志依然会自动滚动到底部，功能完全无效。检查并修复此bug



