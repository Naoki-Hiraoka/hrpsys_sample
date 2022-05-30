# hrpsys_sample

Terminal1
```
$ rtmlaunch hrpsys_sample SampleRobot.launch
```

Terminal2
```
$ roscd hrpsys_sample/scripts
$ ipython -i ./setup.py
$ servoOn()
$ setResetPose()
$ setStAbcParameters()
```

Terminal3
```
rosservice call /Crane/lift False
```

Terminal2
```
$ abc_svc.startAutoBalancer(["rleg","lleg"])
$ st_svc.startStabilizer()
$ abc_svc.goPos(1,0,0)
```