from arctos_arm import ArctosArm

with ArctosArm() as a:
    a.move_joint(3, -40)
    a.move_joint(2, 40)
    a.move_joint(1, 20)


