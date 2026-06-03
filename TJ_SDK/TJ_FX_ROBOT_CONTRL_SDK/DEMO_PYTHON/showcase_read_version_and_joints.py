import sys
import os
import time
import logging
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS

'''#################################################################
该 DEMO 读取控制器版本、SDK 版本及双臂关节反馈角度（带单位打印）

使用逻辑
    初始化订阅结构体与机器人接口
    连接控制器并清错
    确认 UDP 数据通道正常
    读取 VERSION 与 SDK 版本
    订阅并打印 A/B 臂各 7 关节角度（deg / rad）
    释放连接
'''#################################################################

ROBOT_IP = '192.168.1.190'
JOINT_NAMES = [f'J{i + 1}' for i in range(7)]

logging.basicConfig(format='%(message)s')
logger = logging.getLogger('read_version_joints')
logger.setLevel(logging.INFO)


def format_ctrl_version(version: int) -> str:
    """将 100335 解析为 1003_35。"""
    major = version // 100
    minor = version % 100
    return f'{major}_{minor:02d}'


def wait_udp_ready(robot: Marvin_Robot, dcss: DCSS, retries: int = 5) -> bool:
    """确认帧序号刷新，UDP 通道可用。"""
    frame_update = None
    for _ in range(retries):
        sub_data = robot.subscribe(dcss)
        serial = sub_data['outputs'][0]['frame_serial']
        if serial != 0 and serial != frame_update:
            return True
        frame_update = serial
        time.sleep(0.01)
    return False


def print_joint_angles(arm_label: str, joints_deg: list) -> None:
    """打印关节角，单位 deg 与 rad。"""
    logger.info(f'--- {arm_label} 臂关节反馈 ---')
    for name, deg in zip(JOINT_NAMES, joints_deg):
        rad = math.radians(deg)
        logger.info(f'  {name}: {deg:8.3f} deg  ({rad:8.5f} rad)')


def main() -> None:
    dcss = DCSS()
    robot = Marvin_Robot()

    if robot.connect(ROBOT_IP) == 0:
        logger.error('连接失败，端口可能被占用')
        sys.exit(1)

    robot.check_error_and_clear(dcss)

    if not wait_udp_ready(robot, dcss):
        logger.error('UDP 数据通道未就绪，请检查网络/防火墙')
        robot.release_robot()
        sys.exit(1)

    logger.info('机器人连接成功')

    # SDK 大版本
    sdk_ver = robot.SDK_version()
    logger.info(f'SDK 版本: {sdk_ver}')

    # 控制器固件版本
    ret, ctrl_ver = robot.get_param('int', 'VERSION')
    if ret == 0:
        logger.info(
            f'控制器版本: {ctrl_ver}  (解析: {format_ctrl_version(ctrl_ver)})'
        )
    else:
        logger.warning(f'读取 VERSION 失败, ret={ret}')

    # 订阅关节反馈（单位：deg）
    sub_data = robot.subscribe(dcss)
    print_joint_angles('A', sub_data['outputs'][0]['fb_joint_pos'])
    print_joint_angles('B', sub_data['outputs'][1]['fb_joint_pos'])

    robot.release_robot()
    logger.info('已释放连接')


if __name__ == '__main__':
    main()
