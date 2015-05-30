import numpy as np
from openravepy import *
from myarm_const import *
import trajoptpy
import trajoptpy.kin_utils as ku
import json
import time

env = Environment()
env.StopSimulation()
env.SetViewer('qtcoin')
env.Load("../models/arm.xml")

robot = env.GetRobots()[0]
manip = robot.GetActiveManipulator()
j = [1.57, 3.1, 2.1, 1.57, 1.57, 1.57]
robot.SetDOFValues(joint_start, manip.GetArmIndices())
robot.SetDOFValues([GRIPPER_OPEN], manip.GetGripperIndices())

joint_target = [1.57, 3, 2, 2, 2, 3]

request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : "grip",
    "start_fixed" : True
  },
  "costs" : [
  {
    "type" : "joint_vel",
    "params": {"coeffs" : [1]}
  },
  {
    "type" : "collision",
    "params" : {
      "coeffs" : [20],
      "dist_pen" : [0.025]
    },    
  }
  ],
  "constraints" : [
  {
    "type" : "joint",
    "params" : {"vals" : joint_target}
  }
  ],
  "init_info" : {
      "type" : "straight_line",
      "endpoint" : joint_target
  }
}
s = json.dumps(request)
prob = trajoptpy.ConstructProblem(s, env)
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob)
t_elapsed = time.time() - t_start
print result
print "optimization took %.3f seconds"%t_elapsed

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs()
assert traj_is_safe(result.GetTraj(), robot)

traj = result.GetTraj()
for t in traj:
	robot.SetDOFValues(t, manip.GetArmIndices())
	time.sleep(0.25)
