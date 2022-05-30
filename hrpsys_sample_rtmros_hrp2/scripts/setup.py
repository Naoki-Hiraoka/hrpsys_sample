from hrpsys import rtm
rtm.nshost = "localhost"
rtm.nsport = "15005"
rtm.initCORBA()

from hrpsys.OpenHRP import *
import OpenHRP

rh_svc = rtm.findService(rtm.findRTC("rh"),"RobotHardwareService","RobotHardwareService","service0")._narrow(OpenHRP.RobotHardwareService)
seq_svc = rtm.findService(rtm.findRTC("seq"),"SequencePlayerService","SequencePlayerService","service0")._narrow(OpenHRP.SequencePlayerService)
sh_svc = rtm.findService(rtm.findRTC("sh"),"StateHolderService","StateHolderService","service0")._narrow(OpenHRP.StateHolderService)
abc_svc = rtm.findService(rtm.findRTC("abc"),"AutoBalancerService","AutoBalancerService","service0")._narrow(OpenHRP.AutoBalancerService)
st_svc = rtm.findService(rtm.findRTC("st"),"StabilizerService","StabilizerService","service0")._narrow(OpenHRP.StabilizerService)
ic_svc = rtm.findService(rtm.findRTC("ic"),"ImpedanceControllerService","ImpedanceControllerService","service0")._narrow(OpenHRP.ImpedanceControllerService)
kf_svc = rtm.findService(rtm.findRTC("kf"),"KalmanFilterService","KalmanFilterService","service0")._narrow(OpenHRP.KalmanFilterService)

import time
import math
def servoOn(jname='all', tm=3):
    sh_svc.goActual()
    time.sleep(0.1)
    if jname == 'all':
        rh_svc.power(jname, OpenHRP.RobotHardwareService.SWITCH_ON)
    time.sleep(0.1)
    rh_svc.setServoGainPercentage("all", 100.0)
    rh_svc.servo(jname, OpenHRP.RobotHardwareService.SWITCH_ON)
    time.sleep(2)
    return True

def servoOff(jname='all'):
    rh_svc.servo('all', OpenHRP.RobotHardwareService.SWITCH_OFF)
    time.sleep(0.2)
    if jname == 'all':
        rh_svc.power('all', OpenHRP.RobotHardwareService.SWITCH_OFF)
    return True

def setResetPose():
    seq_svc.setJointAngles([-8.002327e-06,0.000427,-0.244732,0.676564,-0.431836,-0.000427,-8.669072e-06,0.000428,-0.244735,0.676565,-0.431834,-0.000428,0.0,0.0,0.0,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]+[0.0]*4, 5.0)
    return True

def setStAbcIcParametersJAXON():
    # abc setting
    abcp=abc_svc.getAutoBalancerParam()[1]
    #abcp.default_zmp_offsets=[[0.015, 0.0, 0.0], [0.015, 0.0, 0.0], [0, 0, 0], [0, 0, 0]];
    abcp.default_zmp_offsets=[[0.0, 0.01, 0.0], [0.0, -0.01, 0.0], [0, 0, 0], [0, 0, 0]];
    abcp.move_base_gain=0.8
    abc_svc.setAutoBalancerParam(abcp)
    # kf setting
    kfp=kf_svc.getKalmanFilterParam()[1]
    kfp.R_angle=1000
    kf_svc.setKalmanFilterParam(kfp)
    # st setting
    stp=st_svc.getParameter()
    #stp.st_algorithm=OpenHRP.StabilizerService.EEFM
    #stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
    stp.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
    stp.emergency_check_mode=OpenHRP.StabilizerService.CP # enable EmergencyStopper for JAXON @ 2015/11/19
    stp.cp_check_margin=[0.05, 0.045, 0, 0.095]
    stp.k_brot_p=[0, 0]
    stp.k_brot_tc=[1000, 1000]
    #stp.eefm_body_attitude_control_gain=[0, 0.5]
    stp.eefm_body_attitude_control_gain=[0.5, 0.5]
    stp.eefm_body_attitude_control_time_const=[1000, 1000]
    stp.eefm_rot_damping_gain = [[60, 60, 1e5],
                                 [60, 60, 1e5],
                                 [20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5],
                                 [20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5]]
    stp.eefm_pos_damping_gain = [[33600, 33600, 9000],
                                 [33600, 33600, 9000],
                                 [3500*1.6*6*0.8, 3500*1.6*6*0.8, 3500*1.6*1.1*1.5*0.8],
                                 [3500*1.6*6*0.8, 3500*1.6*6*0.8, 3500*1.6*1.1*1.5*0.8]]
    stp.eefm_swing_pos_damping_gain = stp.eefm_pos_damping_gain[0]
    stp.eefm_swing_rot_damping_gain = stp.eefm_rot_damping_gain[0]
    stp.eefm_rot_compensation_limit = [math.radians(10), math.radians(10), math.radians(10), math.radians(10)]
    stp.eefm_pos_compensation_limit = [0.025, 0.025, 0.050, 0.050]
    stp.eefm_swing_damping_force_thre=[200]*3
    stp.eefm_swing_damping_moment_thre=[15]*3
    stp.eefm_use_swing_damping=True
    stp.eefm_ee_error_cutoff_freq=20.0
    stp.eefm_swing_rot_spring_gain=[[1.0, 1.0, 1.0]]*4
    stp.eefm_swing_pos_spring_gain=[[1.0, 1.0, 1.0]]*4
    stp.eefm_ee_moment_limit = [[90.0,90.0,1e4], [90.0,90.0,1e4], [1e4]*3, [1e4]*3]
    stp.eefm_rot_time_const = [[1.5/1.1, 1.5/1.1, 1.5/1.1]]*4
    stp.eefm_pos_time_const_support = [[3.0/1.1, 3.0/1.1, 1.5/1.1]]*4
    stp.eefm_wrench_alpha_blending=0.7
    stp.eefm_pos_time_const_swing=0.06
    stp.eefm_pos_transition_time=0.01
    stp.eefm_pos_margin_time=0.02
    # foot margin param
    ## Leptrino force sensor foot : mechanical param is => inside 0.07, front 0.12, rear 0.11
    stp.eefm_leg_inside_margin=0.065
    stp.eefm_leg_outside_margin=0.065
    stp.eefm_leg_front_margin=0.115
    stp.eefm_leg_rear_margin=0.105
    rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_inside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_outside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_outside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_inside_margin])]
    lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_outside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_inside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_inside_margin]),
                    OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_outside_margin])]
    rarm_vertices = rleg_vertices
    larm_vertices = lleg_vertices
    stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
    stp.eefm_cogvel_cutoff_freq = 4.0
    stp.eefm_k1=[-1.48412,-1.48412]
    stp.eefm_k2=[-0.486727,-0.486727]
    stp.eefm_k3=[-0.198033,-0.198033]
    st_svc.setParameter(stp)
    # Abc setting
    #gg=self.abc_svc.getGaitGeneratorParam()[1]
    #gg.stride_parameter=[0.1,0.05,10.0]
    #gg.default_step_time=1.0
    #self.abc_svc.setGaitGeneratorParam(gg)
    gg=abc_svc.getGaitGeneratorParam()[1]
    gg.default_step_time=1.2
    gg.default_step_height=0.065
    #gg.default_double_support_ratio=0.32
    gg.default_double_support_ratio=0.35
    #gg.stride_parameter=[0.1,0.05,10.0]
    #gg.default_step_time=1.0
    #gg.swing_trajectory_delay_time_offset=0.35
    #gg.swing_trajectory_delay_time_offset=0.2
    gg.swing_trajectory_delay_time_offset=0.15
    gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
    gg.swing_trajectory_final_distance_weight=3.0
    gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
    gg.toe_pos_offset_x = 1e-3*117.338;
    gg.heel_pos_offset_x = 1e-3*-116.342;
    gg.toe_zmp_offset_x = 1e-3*117.338;
    gg.heel_zmp_offset_x = 1e-3*-116.342;
    gg.optional_go_pos_finalize_footstep_num=1
    abc_svc.setGaitGeneratorParam(gg)
    # Ic setting
    limbs = ['rarm', 'larm']
    for l in limbs:
        icp = ic_svc.getImpedanceControllerParam(l)[1]
        icp.D_p = 600
        icp.D_r = 200
        ic_svc.setImpedanceControllerParam(l, icp)
