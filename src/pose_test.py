import numpy as np
from openravepy import *
from myarm_const import *

import trajoptpy
import trajoptpy.kin_utils as ku
from trajoptpy.check_traj import traj_is_safe

import json
import time

env = Environment()
env.StopSimulation()
env.SetViewer('qtcoin')
env.Load("arm/arm.xml")

robot = env.GetRobots()[0]
manip = robot.GetActiveManipulator()

joint_start = [1.57, 3.1, 2.1, 1.57, 1.57, 1.57]
robot.SetDOFValues(joint_start, manip.GetArmIndices())
robot.SetDOFValues([GRIPPER_OPEN_CODE], manip.GetGripperIndices())

ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
	ikmodel.autogenerate()
ikmodel.setrobot()



vec_target = [0.125, -0.0274, 0.990, 0.052, -0.078, -0.898, 0.263]
quat_target, xyz_target = vec_target[:4], vec_target[4:]
mat_target = matrixFromPose(vec_target)

manip = robot.GetManipulator('grip')
init_joint_target = ku.ik_for_link(mat_target, manip, "Palm",
    filter_options = IkFilterOptions.CheckEnvCollisions)
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
    "name" : "cont_coll",
    "params" : {
      "continuous" : True,
      "coeffs" : [20],
      "dist_pen" : [0.025]
    },    
  }
  ],
  "constraints" : [
  {
    "type" : "pose",
    "params" : {"xyz" : xyz_target,
                "wxyz" : quat_target,
                "link" : "Palm",
                "timestep" : 9
            }
  }
  ],
  "init_info" : {
      "type" : "straight_line",
      "endpoint" : init_joint_target.tolist()
  }
}
s = json.dumps(request)
prob = trajoptpy.ConstructProblem(s, env)
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob)
t_elapsed = time.time() - t_start

print "optimization took %.3f seconds"%t_elapsed

prob.SetRobotActiveDOFs()
assert traj_is_safe(result.GetTraj(), robot)

traj = result.GetTraj()
for t in traj:
	robot.SetDOFValues(t, manip.GetArmIndices())
	time.sleep(0.5)
