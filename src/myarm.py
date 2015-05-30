import numpy as np
from openravepy import *
from myarm_utils import *
import trajoptpy
import trajoptpy.kin_utils as ku
import serial
import json
import time

class MyArm():

	def __init__(self, path="../models/", bodies=["arm.xml","table.xml","object.xml"], view=True, connect=False):
		e = Environment()
		e.StopSimulation()
		for b in bodies:
			if not e.Load(path + b):
				raise RuntimeError(b + " not found")
		robot = e.GetRobots()[0]
		manip = robot.GetActiveManipulator()
		homeState =  [1.57, 1.9, 2.3, 1.25, 1.57, 1.4]
		robot.SetDOFValues(homeState, manip.GetArmIndices())
		self.env, self.robot, self.manip, self.homeJoints = e, robot, manip, homeState
		
		if connect:
			self.attachPort()
		if view:
			e.SetViewer('qtcoin')
		self.openGrip()

	def home(self):
		self.sendJointState(self.homeJoints)
	
	def back(self):
		if hasattr(self, "reverse"):
			self.followTraj(self.reverse)
	
	def followTraj(self, traj, delay=0.05):
		self.reverse = []
		for step in traj:
			self.reverse = [step] + self.reverse
			self.sendJointState(step)
			self.robot.SetDOFValues(step, self.manip.GetArmIndices())
			time.sleep(delay)

	def moveJoint(self, m, p):
		cmd = str(m) + ',' + str(p) + ','
		self.port.write(cmd)

	def sendJointState(self, joints):
		if len(joints) == len(self.manip.GetArmIndices()):
			self.robot.SetDOFValues(joints, self.manip.GetArmIndices())
			if hasattr(self, 'port'):
				joints =  map(lambda x: int(round(x)), np.degrees(joints))
				for j in range(len(joints)):
					self.moveJoint(j, joints[j])
		else:
			print "wrong number of joints"

	def plotGrip(self):
		self.P = self.env.plot3(points=self.manip.GetTransform().dot(O)[:3],pointsize=15)
		self.P.SetShow(True)

	def jointState(self):
		return self.robot.GetDOFValues(self.manip.GetArmIndices())	

	def readMsg(self):
		if hasattr(self, 'port'):
			msg = '';
			c = self.port.read()
			while c != '':
				msg += c;
				c = self.port.read()
			print msg
			
	def attachPort(self, name='/dev/ttyACM0', baud=9600):
		self.port = serial.Serial(name, baud, timeout=1)
		self.readMsg()
		atexit.register(self.closePort)
		self.sendJointState(self.homeJoints)

	def closePort(self):
		if hasattr(self, 'port'):
			self.port.close()
			print "closed serial connection"

	def openGrip(self):
		if hasattr(self, 'port'):
			self.moveJoint(GRIPPER_INDEX, 0)
		self.robot.SetDOFValues([GRIPPER_OPEN_RAD], self.manip.GetGripperIndices())

	def closeGrip(self):
		if hasattr(self, 'port'):
			self.moveJoint(GRIPPER_INDEX, 1)
		self.robot.SetDOFValues([GRIPPER_CLOSED_RAD], self.manip.GetGripperIndices())

	def computeTrajToPose(self,target):
		params = myIK(target)
		
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
		    "params" : {"xyz" : params['xyz'].tolist(),
				"wxyz" : params['quat'].tolist(),
				"link" : "Palm",
				"timestep" : 9
			    }
		  }
		  ],
		  "init_info" : {
		      "type" : "straight_line",
		      "endpoint" : params['joints'].tolist()
		  }
		}
		s = json.dumps(request)
		prob = trajoptpy.ConstructProblem(s, self.env)
		t_start = time.time()
		result = trajoptpy.OptimizeProblem(prob)
		t_elapsed = time.time() - t_start
		print "optimization took %.3f seconds"%t_elapsed
		return result.GetTraj(), params

if __name__ == "__main__":
	arm = MyArm(view=True, connect=True)
	obj = arm.env.GetKinBody("Object")

	x,y,z = obj.GetTransform()[:3,3]
	target = np.array([0, 0, 1, 0, 1, 0, x, y, z])


	traj, params = arm.computeTrajToPose(target)
	arm.followTraj(traj)

