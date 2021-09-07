#!/usr/bin/env python
# -*- coding: utf-8 -*-
from gatitolabs.arm import ArmSkill
if __name__ == "__main__":
    
    arm = ArmSkill()
    print(arm.get_joint_names())
    arm.set_angles_rad([0.1,0.1,0.1,0.1])
    arm.sleep(1)
    arm.set_angles_rad([3,-3,3,3])
    arm.sleep(2)
    arm.set_neutral()
    arm.sleep(1)


