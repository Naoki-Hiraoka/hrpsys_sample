from hrpsys import rtm
rtm.nshost = "localhost"
rtm.nsport = "15005"
rtm.initCORBA()

from hrpsys.OpenHRP import *
import OpenHRP

rh_svc = rtm.findService(rtm.findRTC("rh"),"RobotHardwareService","RobotHardwareService","service0")._narrow(OpenHRP.RobotHardwareService)
seq_svc = rtm.findService(rtm.findRTC("seq"),"SequencePlayerService","SequencePlayerService","service0")._narrow(OpenHRP.SequencePlayerService)
sh_svc = rtm.findService(rtm.findRTC("sh"),"StateHolderService","StateHolderService","service0")._narrow(OpenHRP.StateHolderService)

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
    seq_svc.setJointAngles([0.000000, -0.036652,  0.000000,  0.078540, -0.041888,  0.000000,  0.174533, -0.003491, 0.000000, -1.570796,  0.000000,  0.000000,  0.000000,  0.000000, -0.036652,  0.000000,  0.078540, -0.041888, 0.000000,  0.174533, -0.003491,  0.000000, -1.570796,  0.000000,  0.000000,  0.000000,  0.00000,  0.000000,  0.000000], 5.0)
    return True
