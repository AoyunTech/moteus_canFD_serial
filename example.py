from moteus_fdcan_adapter import Controller
from moteus_fdcan_adapter import MoteusReg
import time
import math
from kinematics import Kinematics


def main():
    controller_shank = Controller(controller_ID = 8)
    controller_thigh = Controller(controller_ID = 7)
    response_data_c1 = controller_shank.get_data()
    robot_shank_rot = response_data_c1[MoteusReg.MOTEUS_REG_POSITION]
    response_data_c2 = controller_thigh.get_data()
    robot_thigh_rot = response_data_c2[MoteusReg.MOTEUS_REG_POSITION]
    kinematics = Kinematics(robot_shank_rot, robot_thigh_rot)

    freq=300

    while True:
        freq_measure_time = time.time()
        phase = (time.time() * 16) % (2*math.pi)

        z = 190+50*math.sin(phase)
        x = -40+155*math.cos(phase)

        if kinematics.if_ik_possible(x, z):
            robot_thigh_rot_calculated, robot_shank_rot_calculated = kinematics.ik(x, z)

            response_data_c1 = controller_shank.get_data()
            robot_shank_rot_measured = response_data_c1[MoteusReg.MOTEUS_REG_POSITION]
            response_data_c2 = controller_thigh.get_data()
            robot_thigh_rot_measured = response_data_c2[MoteusReg.MOTEUS_REG_POSITION]

            print(f'pos c1: {robot_thigh_rot_measured*360/6:.2f} pos c2: {robot_shank_rot_measured*360/6:.2f}')


            controller_thigh.set_position(position=robot_thigh_rot_calculated, max_torque=1, kd_scale=0.5, kp_scale=1)
            controller_shank.set_position(position=robot_shank_rot_calculated, max_torque=1, kd_scale=0.5, kp_scale=1)

        # response_data_c1 = controller_knee.get_data()
        # robot_knee_rot_org = response_data_c1[MoteusReg.MOTEUS_REG_POSITION]
        # knee_deg_org = kinematics.rad_to_deg(kinematics.robot_to_rad_for_knee(robot_knee_rot_org))
        #
        # response_data_c2 = controller_hip.get_data()
        # robot_hip_rot_org= response_data_c2[MoteusReg.MOTEUS_REG_POSITION]
        # hip_deg_org = kinematics.rad_to_deg(kinematics.robot_to_rad_for_hip(robot_hip_rot_org))
        #
        # x, z = kinematics.fk(robot_hip_rot_org, robot_knee_rot_org)
        # print(kinematics.if_ik_possible(x, z))
        # robot_hip_rot_processed, robot_knee_rot_processed = kinematics.ik(x, z)
        #
        # knee_deg_processed = kinematics.rad_to_deg(kinematics.robot_to_rad_for_knee(robot_knee_rot_processed))
        # hip_deg_processed = kinematics.rad_to_deg(kinematics.robot_to_rad_for_hip(robot_hip_rot_processed))
        #
        # print(f'knee: {knee_deg_org:.2f} / {knee_deg_processed:.2f}, hip: {hip_deg_org:.2f} / {hip_deg_processed:.2f}, X: {x:.2f}, Z: {z:.2f}')


        sleep = (1/freq)-(time.time()-freq_measure_time)
        if sleep < 0: sleep = 0
        time.sleep(sleep)
        #print(f'freq: {1 / (time.time() - freq_measure_time):.1f}')

if __name__ == '__main__':
    main()
