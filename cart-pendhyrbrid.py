import numpy as np
import trep
import trep.discopt
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import matplotlib.pyplot as plt
import time

# set mass, length, and gravity:
DT = 1./30.
M = 0.1 #kg
L = 1.0 # m
B = 0.1 # damping
g = 9.81 #m/s^2
MAXSTEP = 35.0 #m/s^2
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"

lqrflag = False

# define initial config and velocity
q0 = np.array([3./4.*np.pi, 0.]) # x = [theta, xc]
dq0 = np.array([0., 0.])

# define time parameters:
t0 = 0.0
tf = 15.0

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

def proj_func(x):
    x[0] = np.fmod(x[0]+np.pi, 2.0*np.pi)
    if(x[0] < 0):
        x[0] = x[0]+2.0*np.pi
    x[0] = x[0] - np.pi


def build_sac_control(sys):
    sacsyst = sactrep.Sac(sys)
    sacsyst.T = 0.5
    sacsyst.lam = -20
    sacsyst.maxdt = 0.2
    sacsyst.ts = DT
    sacsyst.usat = [[MAXSTEP, -MAXSTEP]]
    sacsyst.calc_tm = DT
    sacsyst.u2search = False
    sacsyst.Q = np.diag([200,10,0,0]) # th, x, thd, xd
    sacsyst.P = np.diag([0,0,0,0])
    sacsyst.R = 0.3*np.identity(1)
    sacsyst.set_proj_func(proj_func)
    return sacsyst

####SETUP#####
system= build_system()
sacsys = build_sac_control(system)
mvi = trep.MidpointVI(system)
mvi.initialize_from_state(t0, q0, np.array([0.]))
sacsys.init()
#########

#def build_LQR_control():
qBar = np.array([0.,0.]) # Desired configuration
Q = np.diag([1,0,1,0]) # Cost weights for states
R = 0.1*np.eye(1) # Cost weights for inputs
TVec = np.arange(t0, tf+DT, DT) # Initialize discrete time vector
dsys = trep.discopt.DSystem(mvi, TVec) # Initialize discrete system
xBar = dsys.build_state(Q=qBar,p = np.zeros(system.nQd)) # Create desired state configuration
Qd = np.zeros((len(TVec), dsys.system.nQ)) # Initialize desired configuration trajectory
thetaIndex = dsys.system.get_config('theta').index # Find index of theta config variable
xcIndex = dsys.system.get_config('yc').index
for i,t in enumerate(TVec):
    Qd[i, thetaIndex] = qBar[0] # Set desired configuration trajectory
    Qd[i, xcIndex] = qBar[1]
    (Xd, Ud) = dsys.build_trajectory(Qd) # Set desired state and input trajectory
Qk = lambda k: Q # Create lambda function for state cost weights
Rk = lambda k: R # Create lambda function for input cost weights
KVec = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk) #Solve for linear controller gain
KStabilize = KVec[0] # use first value to approx infinite-horizon optimal controller gain
dsys.set(np.array([q0[0],q0[1], 0.,0.]), np.array([0.]), 0)
xTilde = 0.*dsys.xk
#    return 

# run loop:
q = np.array((system.q[0], system.q[1],
               system.dq[0], system.dq[1]))
u = [sacsys.controls[0]]
T = [sacsys.time]
Q = [system.q]

while system.t < tf:
    if Q[-1][0] < 0.01 and -0.01 < Q[-1][0]:
        lqrflag = True
        print "FLAG!"
        mvi.initialize_from_configs(T[-3],Q[-3],T[-2],Q[-2])
        dsys.set(np.array([sacsys.q[0],sacsys.q[1],sacsys.dq[0],sacsys.dq[1]]),\
                 np.array([sacsys.controls[0]]), round(T[-1]*30))
    else:
    	lqrflag=False
        
    if lqrflag == True:
        xTilde = dsys.xk-xBar # Compare to desired state
        ulqr = -np.dot(KStabilize, xTilde) # Calculate input
 	if ulqr[0] >35: ulqr[0]=35
        if ulqr[0] < -35: ulqr[0] = -35 
        dsys.step(ulqr) # Step the system forward by one time step
	u.append(ulqr[0])
    else:
        sacsys.step()
	u.append(sacsys.controls[0])
    t_app = sacsys.t_app[1]-sacsys.t_app[0]
    xcalc = system.q[0]+(system.dq[0]*t_app) + (0.5*sacsys.controls[0]*t_app*t_app)
        
    q = np.vstack((q, np.hstack((system.q[0], system.q[1],
                                 system.dq[0], system.dq[1]))))
    
    T.append(system.t)
    qtemp = system.q
    proj_func(qtemp)
    Q = np.vstack((Q,qtemp))
    if np.abs(system.t%1)<DT:
        print "time = ",system.t
        
# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])


plt.plot(T,Q)
plt.plot(T,u)
plt.legend(['th','x','u'])
#plt.axis([0,tf,-10,10])
plt.show()    
np.savetxt("x_py.csv", q, fmt="%9.6f", delimiter=",")
np.savetxt("U_py.csv", u, fmt="%9.6f", delimiter=",")

