# hrpsys_sample

Terminal1
```
$ rtmlaunch hrpsys_sample SampleRobot_choreonoid.launch
```

Terminal2
```
$ rtmlaunch hrpsys_sample SampleRobot_hrpsys.launch
```
(Terminal1のchoreonoidが起動して/clockをpublishし始めた後にHrpsysSeqStateROSBridgeを立ち上げないと、HrpsysSeqStateROSBridgeが勝手に/clockをpublishし始めてしまう.)

Terminal3
```
$ roscd hrpsys_sample/scripts
$ ipython -i ./setup.py
$ servoOn()
$ setResetPose()
$ setStAbcParameters()
$ abc_svc.startAutoBalancer(["rleg","lleg"])
```

Terminal4
```
rosservice call /Crane/lift False
```

Terminal3
```
$ st_svc.startStabilizer()
$ abc_svc.goPos(1,0,0)
```