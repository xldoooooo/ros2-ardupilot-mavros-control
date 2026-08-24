# ===================== 帧头与长度 =====================
SYNC_SEND = bytes([0xA9, 0x5B])   # 上位机发送帧头
SYNC_RECV = bytes([0xB5, 0x9A])   # 云台返回帧头
SEND_PACKET_LEN = 40              # 发送包总字节数
RECV_PACKET_LEN = 26              # 返回包总字节数

# ===================== 命令码 =====================
CMD_NONE = 0       # 无命令
CMD_CALIBRATE = 1  # 陀螺仪校准
CMD_START = 2      # 启动云台
CMD_STOP = 3       # 停止云台
CMD_MANUAL = 4     # 手动控制
CMD_POINT = 5      # 指点平移

# ===================== 工作模式 =====================
WK_MODE_FOLLOW = 0  # 跟随模式
WK_MODE_LOCK = 1    # 锁定模式
WK_MODE_FPV = 2     # FPV模式



# ===================== 预设工作模式常量（可移至 protocol_defs.py） =====================
PRESET_FPV_ANGLE = "fpv_angle"         # FPV模式-角度控制（FPVM-ANGL）
PRESET_PITCH_LOCK_ANGLE = "plck_angle" # 俯仰锁定模式-角度控制（PLCK-ANGL）
PRESET_HORIZON_ANGLE = "hori_angle"    # 地平线模式-角度控制（HORI-ANGL）
PRESET_HORIZON_RATE = "hori_rate"      # 地平线模式-速率控制（HORI-RATE）
PRESET_LOCK_RATE = "lock_rate"         # 锁定模式-速率控制（LOCK-RATE）
PRESET_ONE_KEY_HOME = "goto_zero"      # 一键回中模式（GOTO-ZERO）
PRESET_ONE_KEY_LOOK_DOWN = "look_down" # 一键俯拍模式（LOOK-DOWN）

# ===================== 控制模式 =====================
OP_TYPE_ANGLE = 0       # 角度控制
OP_TYPE_RATIO_SPEED = 1 # 比例角速度控制
OP_TYPE_REAL_SPEED = 2  # 真实角速度控制

# ===================== 轴索引 =====================
AXIS_ROLL = 0   # 滚转轴
AXIS_PITCH = 1  # 俯仰轴
AXIS_YAW = 2    # 偏航轴

# ===================== 云台状态码 =====================
GBC_STAT_IDLE = 0       # 未定义
GBC_STAT_INIT = 1       # 初始化中
GBC_STAT_STOP = 2       # 云台停止
GBC_STAT_PROTECT = 3    # 云台保护
GBC_STAT_MANUAL = 4     # 手动控制
GBC_STAT_POINT = 5      # 指点平移
