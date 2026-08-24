from typing import Optional, Dict
from protocol_defs import *
from gimbal_serial import GimbalSerial
from packet_builder import build_send_packet, parse_response_packet

# ===================== 预设工作模式常量 =====================
PRESET_FPV_ANGLE = "fpv_angle"  # FPV模式-角度控制（FPVM-ANGL）
PRESET_PITCH_LOCK_ANGLE = "plck_angle"  # 俯仰锁定模式-角度控制（PLCK-ANGL）
PRESET_HORIZON_ANGLE = "hori_angle"  # 地平线模式-角度控制（HORI-ANGL）
PRESET_HORIZON_RATE = "hori_rate"  # 地平线模式-速率控制（HORI-RATE）
PRESET_LOCK_RATE = "lock_rate"  # 锁定模式-速率控制（LOCK-RATE）
PRESET_ONE_KEY_HOME = "goto_zero"  # 一键回中模式（GOTO-ZERO）
PRESET_ONE_KEY_LOOK_DOWN = "look_down"  # 一键俯拍模式（LOOK-DOWN）


class GimbalController:
    def __init__(self, port: str, baudrate: int = 115200):
        # 底层通信对象
        self._serial = GimbalSerial(port, baudrate)
        self._serial.set_send_generator(self._generate_send_packet)

        # 控制参数状态
        self._trig_cnt = 0
        self._cmd_value = CMD_MANUAL
        self._fl_sens = 0

        self.axis_go_zero = [0, 0, 0]
        self.axis_wk_mode = [WK_MODE_LOCK, WK_MODE_LOCK, WK_MODE_LOCK]
        self.axis_op_type = [OP_TYPE_ANGLE, OP_TYPE_ANGLE, OP_TYPE_ANGLE]
        self.axis_op_value = [0, 0, 0]

        self.uav_valid = 0
        self.uav_angle = [0.0, 0.0, 0.0]
        self.uav_accel = [0.0, 0.0, 0.0]

        self.cam_vert_fov1x = 0
        self.cam_zoom_value = 0
        self.target_angle = [0.0, 0.0]

        # 模式状态管理
        self.current_mode = "custom"  # 当前模式：custom为自定义配置
        self._mode_controllable = True  # 当前模式是否支持主动控制

        # 缓存解析后的状态
        self._status_cache: Optional[Dict] = None

    def _generate_send_packet(self) -> bytes:
        """发送数据生成回调，供串口层调用"""
        packet = build_send_packet(
            cmd_value=self._cmd_value,
            trig_cnt=self._trig_cnt,
            fl_sens=self._fl_sens,
            axis_go_zero=self.axis_go_zero,
            axis_wk_mode=self.axis_wk_mode,
            axis_op_type=self.axis_op_type,
            axis_op_value=self.axis_op_value,
            uav_valid=self.uav_valid,
            uav_angle=self.uav_angle,
            uav_accel=self.uav_accel,
            cam_vert_fov1x=self.cam_vert_fov1x,
            cam_zoom_value=self.cam_zoom_value,
            target_angle=self.target_angle
        )
        # trig计数自增，保证重复命令可触发
        self._trig_cnt = (self._trig_cnt + 1) % 8
        return packet

    def connect(self) -> bool:
        return self._serial.connect()

    def disconnect(self):
        self._serial.disconnect()

    # ===================== 基础全局命令 =====================
    def send_command(self, cmd_code: int):
        """发送单次命令，执行后恢复手动控制"""
        self._cmd_value = cmd_code
        import time
        time.sleep(0.05)
        self._cmd_value = CMD_MANUAL

    def start_gimbal(self):
        self.send_command(CMD_START)
        print("已发送启动云台命令")

    def stop_gimbal(self):
        self.send_command(CMD_STOP)
        print("已发送停止云台命令")

    def calibrate_gyro(self):
        status = self.get_status()
        if status and not status['tca_flag']:
            print("error：温控未就绪，校准可能失败")
        self.send_command(CMD_CALIBRATE)
        print("已发送陀螺仪校准命令")

    # ===================== 调试辅助函数 =====================
    def print_axis_config(self):
        """打印当前三轴完整配置，用于调试排查"""
        axis_names = ["滚转(ROLL)", "俯仰(PITCH)", "偏航(YAW)"]
        wk_mode_names = {0: "跟随模式", 1: "锁定模式", 2: "FPV模式"}
        op_type_names = {0: "角度控制", 1: "比例角速度", 2: "真实角速度"}

        print("\n========== 当前三轴配置 ==========")
        print(f"当前预设模式: {self.current_mode} | 是否可控: {'是' if self._mode_controllable else '否'}")
        for i in range(3):
            mode_name = wk_mode_names.get(self.axis_wk_mode[i], "未知")
            type_name = op_type_names.get(self.axis_op_type[i], "未知")
            value = self.axis_op_value[i]
            if self.axis_op_type[i] == OP_TYPE_ANGLE:
                value_str = f"{value * 0.01:.2f}°"
            else:
                value_str = f"{value * 0.1:.2f}°/s"
            print(
                f"{axis_names[i]}: 工作模式={mode_name} | 控制类型={type_name} | 控制量={value_str} | 回中标志={self.axis_go_zero[i]}")
        print("==================================\n")

    def print_real_status(self):
        """打印实时云台姿态"""
        status = self.get_status()
        if status:
            print(
                f"实时姿态 → 滚转: {status['cam_angle'][0]:.2f}° | 俯仰: {status['cam_angle'][1]:.2f}° | 偏航: {status['cam_angle'][2]:.2f}°")
        else:
            print("未获取到云台状态")

    # ===================== 预设模式切换（7种整机模式） =====================
    def set_preset_mode(self, preset: str) -> bool:
        """
        切换云台预设工作模式（共7种，匹配C-20T手册定义）
        :param preset: 预设模式常量
        :return: 切换成功返回True
        """
        # 切换前统一复位回中标志位
        self.axis_go_zero = [0, 0, 0]

        if preset == PRESET_FPV_ANGLE:
            for i in range(3):
                self.axis_wk_mode[i] = WK_MODE_FPV
                self.axis_op_type[i] = OP_TYPE_ANGLE
                self.axis_op_value[i] = 0
            self._mode_controllable = True
            print("[预设] 已切换：FPV模式-角度控制（FPVM-ANGL）")

        elif preset == PRESET_PITCH_LOCK_ANGLE:
            self.axis_wk_mode[AXIS_ROLL] = WK_MODE_FOLLOW
            self.axis_op_type[AXIS_ROLL] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_ROLL] = 0
            self.axis_wk_mode[AXIS_PITCH] = WK_MODE_LOCK
            self.axis_op_type[AXIS_PITCH] = OP_TYPE_ANGLE
            self.axis_wk_mode[AXIS_YAW] = WK_MODE_FOLLOW
            self.axis_op_type[AXIS_YAW] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_YAW] = 0
            self._mode_controllable = True
            print("[预设] 已切换：俯仰锁定模式-角度控制（PLCK-ANGL）")

        elif preset == PRESET_HORIZON_ANGLE:
            self.axis_wk_mode[AXIS_ROLL] = WK_MODE_LOCK
            self.axis_op_type[AXIS_ROLL] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_ROLL] = 0  # 滚转0°保持地平线水平
            self.axis_wk_mode[AXIS_PITCH] = WK_MODE_LOCK
            self.axis_op_type[AXIS_PITCH] = OP_TYPE_ANGLE
            self.axis_wk_mode[AXIS_YAW] = WK_MODE_FOLLOW
            self.axis_op_type[AXIS_YAW] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_YAW] = 0
            self._mode_controllable = True
            print("[预设] 已切换：地平线模式-角度控制（HORI-ANGL）")

        elif preset == PRESET_HORIZON_RATE:
            self.axis_wk_mode[AXIS_ROLL] = WK_MODE_LOCK
            self.axis_op_type[AXIS_ROLL] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_ROLL] = 0
            self.axis_wk_mode[AXIS_PITCH] = WK_MODE_LOCK
            self.axis_op_type[AXIS_PITCH] = OP_TYPE_REAL_SPEED
            self.axis_op_value[AXIS_PITCH] = 0
            self.axis_wk_mode[AXIS_YAW] = WK_MODE_FOLLOW
            self.axis_op_type[AXIS_YAW] = OP_TYPE_REAL_SPEED
            self.axis_op_value[AXIS_YAW] = 0
            self._mode_controllable = True
            print("[预设] 已切换：地平线模式-速率控制（HORI-RATE）")

        elif preset == PRESET_LOCK_RATE:
            for i in range(3):
                self.axis_wk_mode[i] = WK_MODE_LOCK
                self.axis_op_type[i] = OP_TYPE_REAL_SPEED
                self.axis_op_value[i] = 0
            self._mode_controllable = True
            print("[预设] 已切换：锁定模式-速率控制（LOCK-RATE）")

        elif preset == PRESET_ONE_KEY_HOME:
            self.axis_wk_mode[AXIS_ROLL] = WK_MODE_LOCK
            self.axis_op_type[AXIS_ROLL] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_ROLL] = 0
            self.axis_wk_mode[AXIS_PITCH] = WK_MODE_LOCK
            self.axis_op_type[AXIS_PITCH] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_PITCH] = 0
            self.axis_wk_mode[AXIS_YAW] = WK_MODE_FOLLOW
            self.axis_op_type[AXIS_YAW] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_YAW] = 0
            self._mode_controllable = False
            print("[预设] 已切换：一键回中模式（GOTO-ZERO），此模式下云台不可控")

        elif preset == PRESET_ONE_KEY_LOOK_DOWN:
            self.axis_wk_mode[AXIS_ROLL] = WK_MODE_LOCK
            self.axis_op_type[AXIS_ROLL] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_ROLL] = 0
            self.axis_wk_mode[AXIS_PITCH] = WK_MODE_LOCK
            self.axis_op_type[AXIS_PITCH] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_PITCH] = int(-90 * 100)  # 竖直向下90°
            self.axis_wk_mode[AXIS_YAW] = WK_MODE_FOLLOW
            self.axis_op_type[AXIS_YAW] = OP_TYPE_ANGLE
            self.axis_op_value[AXIS_YAW] = 0
            self._mode_controllable = False
            print("[预设] 已切换：一键俯拍模式（LOOK-DOWN），此模式下云台不可控")

        else:
            print(f"[错误] 未知预设模式: {preset}")
            return False

        self.current_mode = preset
        return True

    # ===================== 运动控制（默认不改模式，仅修改控制量） =====================
    def set_axis_angle(self, axis: int, angle_deg: float, wk_mode: int = None) -> bool:
        """
        设置指定轴目标角度
        :param axis: 轴索引
        :param angle_deg: 目标角度/偏移角度，单位度
        :param wk_mode: 工作模式，默认None（保持当前模式不变）
        :return: 设置成功返回True
        """
        if not self._mode_controllable:
            print(f"[警告] 当前模式 {self.current_mode} 为不可控模式，角度指令无效")
            return False
        if axis not in (AXIS_ROLL, AXIS_PITCH, AXIS_YAW):
            print(f"[错误] 无效的轴索引: {axis}")
            return False

        # 仅显式传入模式时才修改
        if wk_mode is not None:
            if wk_mode not in (WK_MODE_FOLLOW, WK_MODE_LOCK, WK_MODE_FPV):
                print(f"[错误] 无效的工作模式: {wk_mode}")
                return False
            if wk_mode == WK_MODE_FPV:
                for i in range(3):
                    self.axis_wk_mode[i] = WK_MODE_FPV
                    self.axis_op_type[i] = OP_TYPE_ANGLE
                print("[提示] 已切换为FPV模式，三轴同步生效，仅支持角度控制")
            else:
                self.axis_wk_mode[axis] = wk_mode
                if wk_mode == WK_MODE_FOLLOW and axis in (AXIS_ROLL, AXIS_PITCH):
                    print(f"[警告] 协议不建议滚转/俯仰轴使用跟随模式")

        self.axis_op_type[axis] = OP_TYPE_ANGLE
        self.axis_op_value[axis] = int(angle_deg * 100)
        self.current_mode = "custom"
        return True

    def set_axis_velocity(self, axis: int, speed_dps: float,
                          wk_mode: int = None,
                          op_type: int = OP_TYPE_REAL_SPEED) -> bool:
        """
        设置指定轴转动角速度
        :param axis: 轴索引
        :param speed_dps: 角速度，单位°/s；0则锁定当前角度
        :param wk_mode: 工作模式，默认None（保持当前模式不变）
        :param op_type: 角速度类型，默认真实角速度
        :return: 设置成功返回True
        """
        if not self._mode_controllable:
            print(f"[警告] 当前模式 {self.current_mode} 为不可控模式，角速度指令无效")
            return False
        if axis not in (AXIS_ROLL, AXIS_PITCH, AXIS_YAW):
            print(f"[错误] 无效的轴索引: {axis}")
            return False
        if op_type not in (OP_TYPE_RATIO_SPEED, OP_TYPE_REAL_SPEED):
            print(f"[错误] 无效的角速度控制类型: {op_type}")
            return False

        if wk_mode is not None:
            if wk_mode == WK_MODE_FPV:
                print("[错误] FPV模式不支持角速度控制")
                return False
            if wk_mode not in (WK_MODE_FOLLOW, WK_MODE_LOCK):
                print(f"[错误] 无效的工作模式: {wk_mode}")
                return False
            self.axis_wk_mode[axis] = wk_mode
            if wk_mode == WK_MODE_FOLLOW and axis in (AXIS_ROLL, AXIS_PITCH):
                print(f"[警告] 协议不建议滚转/俯仰轴使用跟随模式")

        self.axis_op_type[axis] = op_type
        self.axis_op_value[axis] = int(speed_dps * 10)

        if op_type == OP_TYPE_RATIO_SPEED:
            print("[提示] 比例角速度依赖相机vert_fov1x和zoom_value参数")

        self.current_mode = "custom"
        return True

    # ===================== 可靠回中函数（角度闭环为主，脉冲为辅） =====================
    def set_axis_go_home(self, axis: int, wait_complete: bool = False, timeout: float = 8.0) -> bool:
        """
        单轴可靠回中（角度持续闭环，保证大角度精准到位）
        :param axis: 轴索引
        :param wait_complete: 是否阻塞等待回中完成
        :param timeout: 等待超时时间，单位秒，默认8秒覆盖满行程
        :return: 到位返回True，超时返回False
        """
        if axis not in (AXIS_ROLL, AXIS_PITCH, AXIS_YAW):
            print(f"[错误] 无效的轴索引: {axis}")
            return False

        import time
        axis_names = ["滚转", "俯仰", "偏航"]

        # 1. 先切锁定模式+角度控制，目标0°，持续闭环本身就是回中
        self.axis_wk_mode[axis] = WK_MODE_LOCK
        self.axis_op_type[axis] = OP_TYPE_ANGLE
        self.axis_op_value[axis] = 0

        # 2. 附加go_zero脉冲，加速触发响应
        self.axis_go_zero[axis] = 0
        time.sleep(0.03)
        self.axis_go_zero[axis] = 1
        time.sleep(0.05)
        self.axis_go_zero[axis] = 0

        if not wait_complete:
            self.current_mode = "custom"
            return True

        # 3. 等待角度收敛到0.5°以内
        start_time = time.time()
        final_angle = 999.0
        while time.time() - start_time < timeout:
            time.sleep(0.1)
            status = self.get_status()
            if status:
                final_angle = status['cam_angle'][axis]
                if abs(final_angle) < 0.5:
                    print(f"[回中成功] {axis_names[axis]}轴已到位，当前角度: {final_angle:.2f}°")
                    self.current_mode = "custom"
                    return True

        # 超时退出
        print(f"[回中超时] {axis_names[axis]}轴当前角度: {final_angle:.2f}°，请检查机械限位或延长超时时间")
        self.current_mode = "custom"
        return False

    def set_all_go_home(self, wait_complete: bool = False, timeout: float = 8.0) -> bool:
        """
        三轴可靠回中
        :param wait_complete: 是否阻塞等待全部回中完成
        :param timeout: 等待超时时间，单位秒
        :return: 全部到位返回True，超时返回False
        """
        import time

        # 全部切锁定模式+角度控制，目标0°
        for i in range(3):
            self.axis_wk_mode[i] = WK_MODE_LOCK
            self.axis_op_type[i] = OP_TYPE_ANGLE
            self.axis_op_value[i] = 0
            self.axis_go_zero[i] = 0

        time.sleep(0.03)

        # 统一脉冲触发
        for i in range(3):
            self.axis_go_zero[i] = 1
        time.sleep(0.05)
        for i in range(3):
            self.axis_go_zero[i] = 0

        if not wait_complete:
            self.current_mode = "custom"
            return True

        # 等待全部收敛
        start_time = time.time()
        final_angles = [999.0, 999.0, 999.0]
        while time.time() - start_time < timeout:
            time.sleep(0.1)
            status = self.get_status()
            if status:
                all_done = True
                for i in range(3):
                    final_angles[i] = status['cam_angle'][i]
                    if abs(final_angles[i]) > 0.5:
                        all_done = False
                        break
                if all_done:
                    print(
                        f"[回中成功] 三轴全部到位，滚转{final_angles[0]:.2f}° 俯仰{final_angles[1]:.2f}° 偏航{final_angles[2]:.2f}°")
                    self.current_mode = "custom"
                    return True

        print(f"[回中超时] 当前角度：滚转{final_angles[0]:.2f}° 俯仰{final_angles[1]:.2f}° 偏航{final_angles[2]:.2f}°")
        self.current_mode = "custom"
        return False

    # ===================== 参数配置 =====================
    def set_fpv_sensitivity(self, sens: int):
        """设置FPV跟随灵敏度，范围[-16, 15]"""
        if WK_MODE_FPV not in self.axis_wk_mode:
            print("[提示] 当前非FPV模式，灵敏度设置暂不生效")
        self._fl_sens = max(-16, min(15, sens))
        print(f"FPV跟随灵敏度已设置为: {self._fl_sens}")

    def set_work_mode(self, axis: int, mode: int) -> bool:
        """单独设置指定轴工作模式"""
        if not self._mode_controllable:
            print(f"[警告] 当前模式 {self.current_mode} 为不可控模式，无法修改工作模式")
            return False
        if axis not in (AXIS_ROLL, AXIS_PITCH, AXIS_YAW):
            print(f"[错误] 无效的轴索引: {axis}")
            return False
        if mode not in (WK_MODE_FOLLOW, WK_MODE_LOCK, WK_MODE_FPV):
            print(f"[错误] 无效的工作模式: {mode}")
            return False

        if mode == WK_MODE_FPV:
            for i in range(3):
                self.axis_wk_mode[i] = WK_MODE_FPV
                if self.axis_op_type[i] != OP_TYPE_ANGLE:
                    self.axis_op_type[i] = OP_TYPE_ANGLE
                    self.axis_op_value[i] = 0
            print("[提示] 已切换为FPV模式，已自动切换为角度控制")
        else:
            self.axis_wk_mode[axis] = mode
            if mode == WK_MODE_FOLLOW and axis in (AXIS_ROLL, AXIS_PITCH):
                print(f"[警告] 协议不建议滚转/俯仰轴使用跟随模式")

        self.current_mode = "custom"
        return True

    # ===================== 状态获取 =====================
    def get_status(self) -> Optional[Dict]:
        """获取最新云台状态（自动解析）"""
        raw_packet = self._serial.get_latest_packet()
        if raw_packet:
            self._status_cache = parse_response_packet(raw_packet)
        return self._status_cache
