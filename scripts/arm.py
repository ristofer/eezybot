#!/usr/bin/env python
# -*- coding: utf-8 -*-
from gatitolabs.arm import ArmSkill
if __name__ == "__main__":
    

    arm = ArmSkill()

    arm.setup()
    print(arm.get_joint_names())
    while(not arm.is_shutdown()):
        arm.set_angles([0.1,0.4,0.4,0.4])


