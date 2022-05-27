#!/usr/bin/python
# -*- coding: utf-8 -*-
import baxter_interface
lhc=baxter_interface.CameraController("left_hand_camera")
#lhc.exposure=80
print(lhc.exposure)


