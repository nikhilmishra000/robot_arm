import numpy as np
from openravepy import *
from scipy.optimize import fmin_l_bfgs_b, curve_fit
from scipy.interpolate import NearestNDInterpolator as interp

GRIPPER_OPEN_RAD, GRIPPER_CLOSED_RAD = np.radians(65), np.radians(0)
GRIPPER_INDEX = 6

O = np.array([-0.09327, 0.97265, 0.0976, 1])

def decomposeQuat(q):
	axang = axisAngleFromQuat(q)
	angle = np.linalg.norm(axang); axis = axang/angle
	return axis, angle

def gripPointDir(q):
	if type(q) == numpy.ndarray:
		m = matrixFromQuat(q)[:3,:3]
	else:
		m = q.GetTransform()[:3,:3]
	return m.dot(np.array([[-1],[0],[0]]))

def gripOpenDir(q):
	if type(q) == numpy.ndarray:
		m = matrixFromQuat(q)[:3,:3]
	else:
		m = q.GetTransform()[:3,:3]
	return m.dot(np.array([[0],[1],[0]]))

def gripperState(m):
	q = poseFromMatrix(m)[:4]
	pointdir = gripPointDir(q)
	opendir = gripOpenDir(q)
	grasppos = m.dot(O)[:3][:,np.newaxis]
	return (np.vstack([pointdir, opendir, grasppos]).T)[0]

def myIK(target, dim=9, w=[1,1,1,1,1,1,15,15,15]):
	e = Environment()
	e.StopSimulation()
	e.Load("arm/arm.xml")
	e.Load("arm/table.xml")
	robot = e.GetRobots()[0]
	manip = robot.GetActiveManipulator()
	data = np.load("arm_reach_8.npy")

	if dim == 9:
		nn = interp(data[:,:9], data[:,9:])
	elif dim == 3:
		nn = interp(data[:,6:9], data[:,9:])


	joint_start = nn(target[np.newaxis])[0].tolist()
	robot.SetDOFValues(joint_start, manip.GetArmIndices())
	robot.SetDOFValues([GRIPPER_OPEN_RAD], manip.GetGripperIndices())



	limits = []
	for j in robot.GetJoints():
		limits.append((j.GetLimits()[0][0], j.GetLimits()[1][0]))
	
	def cost(joints):
		robot.SetDOFValues(joints, manip.GetArmIndices())
		gs = gripperState(manip.GetTransform())

		if dim == 9:
			distCost = np.linalg.norm ( (gs - target) * np.array(w) )
		elif dim == 3:
			distCost = np.linalg.norm( (gs[-3:] - target[-3:]) )

		collCost = 100 * sum([sum([1 if e.CheckCollision(f, o) else 0 for o in e.GetBodies()[1:]]) for f in manip.GetChildLinks()])
		return distCost + collCost

	final, fmin, d = fmin_l_bfgs_b(cost, joint_start, maxfun=500, iprint=10, m=50, approx_grad=True, pgtol=1e-12, factr=1)
	robot.SetDOFValues(final, manip.GetArmIndices())
	finalPose = poseFromMatrix(manip.GetTransform())
	return {"joints": final, "quat": finalPose[:4], "xyz": finalPose[4:], "cost":fmin}

