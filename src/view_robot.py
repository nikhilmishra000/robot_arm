import numpy as np
from openravepy import *
from myarm_const import *




env = Environment()
env.SetViewer('qtcoin')
env.Load('../arm/arm_no_limits.xml')
r = env.GetRobots()[0]
m = r.GetActiveManipulator()

j = [1.57, 3.1, 2.1, 1.57, 1.57, 1.57]
r.SetDOFValues(j, m.GetArmIndices())
r.SetDOFValues([GRIPPER_OPEN_CODE], m.GetGripperIndices())

