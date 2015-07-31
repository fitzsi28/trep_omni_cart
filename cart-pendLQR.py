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
DT = 1./30. # Sampling time
M = 0.1 # Mass of pendulum
L = 1.0 # Length of pendulum
B = 0.1 #damping
g = 9.81
MAXSTEP = 20.
MASSFRAME = "pend_mass"
CARTFRAME = "cart"


q0 = 150.*np.pi/180. # Initial configuration of pendulum
t0 = 0.0 # Initial time
tf = 5.0# Final time

qBar = np.array([0., np.pi, 0.]) # Desired configuration
Q = np.diag([1,1,1,1,1,1]) # Cost weights for states
R = 0.1*np.eye(1) # Cost weights for inputs

system = trep.System() # Initialize system

frames = [
    ty('ys', name='y-stylus', kinematic=True),
    ty('yc',name=CARTFRAME, mass=M), [ 
        rx('theta', name="pendulumShoulder"), [
            tz(-L, name=MASSFRAME, mass=M)]]]
system.import_frames(frames) # Add frames

# Add forces to the system
trep.potentials.Gravity(system, (0, 0, -g)) # Add gravity
trep.forces.Damping(system, B) # Add damping
trep.constraints.PointOnPlane(system, 'y-stylus', (0.,1.0,0.), CARTFRAME)#add a constraint


# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(t0, np.array([q0]), t0+DT, np.array([q0]))

# Create discrete system
TVec = np.arange(t0, tf+DT, DT) # Initialize discrete time vector
dsys = trep.discopt.DSystem(mvi, TVec) # Initialize discrete system
xBar = dsys.build_state(Q=qBar,p = np.zeros(system.nQd)) # Create desired state configuration

# Design linear feedback controller
Qd = np.zeros((len(TVec), dsys.system.nQ)) # Initialize desired configuration trajectory
thetaIndex = dsys.system.get_config('theta').index # Find index of theta config variable
ycIndex = dsys.system.get_config('yc').index
ysIndex = dsys.system.get_config('ys').index
for i,t in enumerate(TVec):
    Qd[i, thetaIndex] = qBar[0] # Set desired configuration trajectory
    Qd[i, ycIndex] = qBar[1]
    Qd[i, ysIndex] = qBar[2]
    (Xd, Ud) = dsys.build_trajectory(Qd) # Set desired state and input trajectory

Qk = lambda k: Q # Create lambda function for state cost weights
Rk = lambda k: R # Create lambda function for input cost weights
KVec = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk) # Solve for linear feedback controller gain
KStabilize = KVec[0] # Use only use first value to approximate infinite-horizon optimal controller gain

# Reset discrete system state
dsys.set(np.array([0.,q0, 0.,0.,0.,0.]), np.array([0.]), 0)

# Simulate the system forward
T = [mvi.t1] # List to hold time values
Q = [mvi.q1] # List to hold configuration values
X = [dsys.xk] # List to hold state values
U = [] # List to hold input values

while mvi.t1 < tf-DT:
    x = dsys.xk # Grab current state
    xTilde = x - xBar # Compare to desired state
    u = -dot(KStabilize, xTilde) # Calculate input
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
