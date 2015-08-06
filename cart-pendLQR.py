# Import necessary python modules
import math
from math import pi
import numpy as np
from numpy import dot
import trep
import trep.discopt
from trep import tx, ty, tz, rx, ry, rz
import pylab

# Build a pendulum system
DT = 1./60. # Sampling time
M = 0.1 # Mass of pendulum
L = 1.0 # Length of pendulum
B = 0.1 #damping
g = 9.81
MAXSTEP = 20.
MASSFRAME = "pend_mass"
CARTFRAME = "cart"


X0 = np.array([0.3,0.,0.,0.])# Initial configuration of pendulum


def build_system():
    sys = trep.System()
    frames = [
        ty('yc',name=CARTFRAME, kinematic=True), [ 
            rx('theta', name="pendulumShoulder"), [
                tz(L, name=MASSFRAME, mass=M)]]]
    sys.import_frames(frames)
    trep.potentials.Gravity(sys, (0,0,-g))
    trep.forces.Damping(sys, B)
    return sys

system = build_system()

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(0.0, X0[0:1], DT, X0[0:1])

def build_lqr_control(sys,mvi, X0):
    t0 = 0.0 
    tf = 30.0
    qBar = np.array([0., 0.]) 
    Q = np.diag([1,1,1,1]) 
    R = 0.1*np.eye(1) 
    TVec = np.arange(t0, tf+DT, DT) 
    dsyst = trep.discopt.DSystem(mvi, TVec) 
    xBar0 = dsyst.build_state(Q=qBar,p = np.zeros(sys.nQd)) 
    Qd = np.zeros((len(TVec), dsyst.system.nQ)) 
    thetaIndex = dsyst.system.get_config('theta').index 
    ycIndex = dsyst.system.get_config('yc').index
    for i,t in enumerate(TVec):
        Qd[i, thetaIndex] = qBar[0] 
        Qd[i, ycIndex] = qBar[1]
        (Xd, Ud) = dsyst.build_trajectory(Qd) 
    Qk = lambda k: Q 
    Rk = lambda k: R 
    KVec = dsyst.calc_feedback_controller(Xd, Ud, Qk, Rk) 
    KStabilize = KVec[0] 
    dsyst.set(X0, np.array([0.]), 0)
    return dsyst, xBar0, KStabilize

dsys,xBar,KStabil = build_lqr_control(system,mvi, X0)
# Simulate the system forward
T = [mvi.t1] # List to hold time values
Q = [mvi.q1] # List to hold configuration values
X = [dsys.xk] # List to hold state values
U = [] # List to hold input values
tf=30
while mvi.t1 < tf-DT:
    x = dsys.xk # Grab current state
    xTilde = x - xBar # Compare to desired state
    u = -dot(KStabil, xTilde) # Calculate input
    dsys.step(u) # Step the system forward by one time step
    T.append(mvi.t1) # Update lists
    Q.append(mvi.q1)
    X.append(x)
    U.append(u)
    if np.abs(mvi.t1%1)<DT:
        print "time = ",mvi.t1

# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])

# Plot results
ax1 = pylab.subplot(211)
pylab.plot(T, X)
pylab.title("Linear Feedback Controller")
pylab.ylabel("X")
pylab.legend(["theta","x","dtheta","dx"])
pylab.subplot(212, sharex=ax1)
pylab.plot(T[1:], U)
pylab.xlabel("T")
pylab.ylabel("U")
pylab.legend(["u"])
pylab.show()
