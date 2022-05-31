# hrpsys_sample

Terminal1
```
$ rtmlaunch hrpsys_sample_rtmros_hrp2 JAXON_JVRC.launch
```

Terminal2
```
$ roscd hrpsys_sample_rtmros_hrp2/scripts
$ ipython -i ./jaxon_jvrc_setup.py
$ servoOn()
$ setResetPose()
$ setStAbcParametersJAXON()
$ abc_svc.startAutoBalancer(["rleg","lleg"])
```

Terminal3
```
rosservice call /Crane/lift False
```

Terminal2
```
$ st_svc.startStabilizer()
$ abc_svc.goPos(1,0,0)
```