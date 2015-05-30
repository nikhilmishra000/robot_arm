import numpy as np
from openravepy import *
from myarm_utils import *
import trajoptpy
import trajoptpy.kin_utils as ku
import time
import pickle

env = Environment()
env.Load('../models/arm.xml')
r = env.GetRobots()[0]
m = r.GetActiveManipulator()


ji = [1.57, 1.57, 2.1, 1.25, 1.57, 1.57]
r.SetDOFValues(ji, m.GetArmIndices())

js = {}
for j in r.GetJoints():
	js[j.GetDOFIndex()] = {0: j.GetLimits()[0][0], 1: j.GetLimits()[1][0]}

t = time.time()
i = 0
nsteps = 8; total = nsteps**6

print "Generating reachability database with",total,"positions"

data = np.zeros([total, 15])
for i0 in np.linspace(js[0][0], js[0][1], nsteps):
	for i1 in np.linspace(js[1][0], js[1][1], nsteps):
		for i2 in np.linspace(js[2][0], js[2][1], nsteps):
			for i3 in np.linspace(js[3][0], js[3][1], nsteps):
				for i4 in np.linspace(js[4][0], js[4][1], nsteps):
					for i5 in np.linspace(js[5][0], js[5][1], nsteps):
						j = np.array([i0,i1,i2,i3,i4,i5])
						r.SetDOFValues(j, m.GetArmIndices())

						gs = gripperState(m.GetTransform())[np.newaxis] 
						data[i,:] = np.hstack([gs, j[np.newaxis]])
						
						i += 1
						if not i % 5000:
							print "computed position " + str(i) + " of " + str(total)
print "Elapsed time: ", time.time() - t
fname = "arm_reach_" + str(nsteps) + ".npy"
np.save(fname, data)
print "Data written to " + fname
