import time
from gimbal_controller import (
    GimbalController,
    PRESET_FPV_ANGLE,
    PRESET_PITCH_LOCK_ANGLE,
    PRESET_HORIZON_ANGLE,
    PRESET_HORIZON_RATE,
    PRESET_LOCK_RATE,
    PRESET_ONE_KEY_HOME,
    PRESET_ONE_KEY_LOOK_DOWN
)
from protocol_defs import AXIS_PITCH, AXIS_ROLL, AXIS_YAW

def set_angle (gimbal: GimbalController, pitch_angle : 20, yaw_angle : 20, preset: str, control: bool = True):
    """单个模式测试封装"""
    print("\n" + "="*50)
    gimbal.set_preset_mode(preset)
    time.sleep(1)  # 等待模式切换稳定
    gimbal.print_axis_config()
    gimbal.print_real_status()

    if control and gimbal._mode_controllable:
        print("\n>>> 执行控制测试")
        # 俯仰轴控制测试
        gimbal.set_axis_angle(AXIS_PITCH, pitch_angle)
        time.sleep(2)
        # gimbal.print_real_status()

        # 偏航轴控制测试
        gimbal.set_axis_angle(AXIS_YAW, yaw_angle)
        time.sleep(2)
        # gimbal.print_real_status()

        # 俯仰回中
        # print("\n>>> 俯仰轴回中测试")
        # gimbal.set_axis_go_home(AXIS_PITCH, wait_complete=True)
        # gimbal.print_real_status()

    time.sleep(1)
    print("="*50 + "\n")

def test_mode_preset(gimbal: GimbalController, preset: str, control: bool = True):
    """单个模式测试封装"""
    print("\n" + "="*50)
    gimbal.set_preset_mode(preset)
    time.sleep(1)  # 等待模式切换稳定
    gimbal.print_axis_config()
    gimbal.print_real_status()

    if control and gimbal._mode_controllable:
        print("\n>>> 执行控制测试")
        # 俯仰轴控制测试
        gimbal.set_axis_angle(AXIS_PITCH, 20)
        time.sleep(2)
        gimbal.print_real_status()

        # 偏航轴控制测试
        gimbal.set_axis_angle(AXIS_YAW, 20)
        time.sleep(2)
        gimbal.print_real_status()

        # 俯仰回中
        print("\n>>> 俯仰轴回中测试")
        gimbal.set_axis_go_home(AXIS_PITCH, wait_complete=True)
        gimbal.print_real_status()

    time.sleep(1)
    print("="*50 + "\n")


def main():
    # 创建控制器
    gimbal = GimbalController(port="/dev/ttyUSB0", baudrate=115200)

    if not gimbal.connect():
        print("串口连接失败，程序退出")
        return

    try:
        print("=== 云台初始化 ===")
        time.sleep(1)
        test_mode_preset(gimbal, PRESET_HORIZON_ANGLE, control=False)
        gimbal.print_real_status()

        while True:
            # 俯仰轴控制测试
            pitch_angle = int(input("请输入pitch_angle："))
            gimbal.set_axis_angle(AXIS_PITCH, pitch_angle)
            time.sleep(1)
            gimbal.print_real_status()

            # 偏航轴控制测试
            yaw_angle = int(input("请输入yaw_angle："))
            gimbal.set_axis_angle(AXIS_YAW, yaw_angle)
            time.sleep(1)
            gimbal.print_real_status()

        # 俯仰回中
        # print("\n>>> 俯仰轴回中测试")
        # gimbal.set_axis_go_home(AXIS_PITCH, wait_complete=True)
        # gimbal.print_real_status()


        # set_angle(gimbal, 30, 30, PRESET_FPV_ANGLE, control=True)
        # test_mode_preset(gimbal, PRESET_HORIZON_ANGLE, control=False)

        # PRESET_FPV_ANGLE = "fpv_angle"  # FPV模式-角度控制（FPVM-ANGL）
        # PRESET_PITCH_LOCK_ANGLE = "plck_angle"  # 俯仰锁定模式-角度控制（PLCK-ANGL）
        # PRESET_HORIZON_ANGLE = "hori_angle"  # 地平线模式-角度控制（HORI-ANGL）
        # PRESET_HORIZON_RATE = "hori_rate"  # 地平线模式-速率控制（HORI-RATE）
        # PRESET_LOCK_RATE = "lock_rate"  # 锁定模式-速率控制（LOCK-RATE）
        # PRESET_ONE_KEY_HOME = "goto_zero"  # 一键回中模式（GOTO-ZERO）
        # PRESET_ONE_KEY_LOOK_DOWN = "look_down"  # 一键俯拍模式（LOOK-DOWN）

        # # ========== 1. FPV模式-角度控制 ==========
        # test_mode_preset(gimbal, PRESET_FPV_ANGLE, control=True)
        #
        # # ========== 2. 俯仰锁定模式-角度控制 ==========
        # test_mode_preset(gimbal, PRESET_PITCH_LOCK_ANGLE, control=True)
        # set_angle(gimbal, 30, 30, PRESET_FPV_ANGLE, control=True)
        #
        # # ========== 3. 地平线模式-角度控制 ==========
        # test_mode_preset(gimbal, PRESET_HORIZON_ANGLE, control=True)

        #
        # # ========== 4. 地平线模式-速率控制 ==========
        # test_mode_preset(gimbal, PRESET_HORIZON_RATE, control=False)
        # if gimbal._mode_controllable:
        #     print(">>> 速率模式转动测试")
        #     gimbal.set_axis_velocity(AXIS_PITCH, 15)
        #     time.sleep(3)
        #     gimbal.set_axis_velocity(AXIS_PITCH, 0)
        #     gimbal.set_axis_velocity(AXIS_YAW, 15)
        #     time.sleep(3)
        #     gimbal.set_axis_velocity(AXIS_YAW, 0)
        #     gimbal.set_axis_velocity(AXIS_ROLL, 15)
        #     time.sleep(3)
        #     gimbal.set_axis_velocity(AXIS_ROLL, 0)
        #     gimbal.print_real_status()
        #
        # # ========== 5. 锁定模式-速率控制 ==========
        # test_mode_preset(gimbal, PRESET_LOCK_RATE, control=False)
        # if gimbal._mode_controllable:
        #     print(">>> 全锁定速率转动测试")
        #     gimbal.set_axis_velocity(AXIS_YAW, 20)
        #     time.sleep(2)
        #     gimbal.set_axis_velocity(AXIS_YAW, 0)
        #     gimbal.print_real_status()
        #
        # # ========== 6. 一键回中模式（不可控） ==========
        test_mode_preset(gimbal, PRESET_ONE_KEY_HOME, control=False)
        time.sleep(2)
        gimbal.print_real_status()
        #
        # # ========== 7. 一键俯拍模式（不可控） ==========
        # test_mode_preset(gimbal, PRESET_ONE_KEY_LOOK_DOWN, control=False)
        # time.sleep(2)
        # gimbal.print_real_status()
        #
        # # ========== 最终：全轴可靠回中测试 ==========
        # print("\n" + "="*50)
        # print("=== 最终三轴回中综合测试 ===")
        # gimbal.set_all_go_home(wait_complete=True, timeout=10.0)
        # gimbal.print_real_status()
        # print("="*50)
        #
        # # 最终状态打印
        # print("\n=== 运行状态 ===")
        # status = gimbal.get_status()
        # if status:
        #     print(f"固件版本: {status['fw_ver']}")
        #     print(f"硬件故障: {status['hw_err']}")
        #     print(f"安装方式: {'立装' if status['inv_flag'] else '吊装'}")
        #     print(f"温控就绪: {'是' if status['tca_flag'] else '否'}")
        #     gimbal.print_real_status()

    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        # 断开前先回中
        print("\n断开连接前执行回中...")
        # gimbal.set_all_go_home(wait_complete=True)
        # test_mode_preset(gimbal, PRESET_ONE_KEY_HOME, control=False)
        time.sleep(1)
        gimbal.disconnect()
        print("串口已关闭")


if __name__ == "__main__":
    main()