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

import time
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
    seq_svc.setJointAngles([-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0], 5.0)
    return True

def setStAbcParameters ():
    # ST parameters
    stp=st_svc.getParameter()
    stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
    #   eefm st params
    stp.eefm_leg_inside_margin=71.12*1e-3
    stp.eefm_leg_outside_margin=71.12*1e-3
    stp.eefm_leg_front_margin=182.0*1e-3
    stp.eefm_leg_rear_margin=72.0*1e-3
    stp.eefm_k1=[-1.39899,-1.39899]
    stp.eefm_k2=[-0.386111,-0.386111]
    stp.eefm_k3=[-0.175068,-0.175068]
    stp.eefm_rot_damping_gain=[[20*1.6*10, 20*1.6*10, 1e5]]*4 # Stiff parameter for simulation
    stp.eefm_pos_damping_gain=[[3500*50, 3500*50, 3500*1.0*5]]*4 # Stiff parameter for simulation
    #   tpcc st params
    stp.k_tpcc_p=[0.2, 0.2]
    stp.k_tpcc_x=[4.0, 4.0]
    stp.k_brot_p=[0.0, 0.0]
    stp.k_brot_tc=[0.1, 0.1]
    st_svc.setParameter(stp)
