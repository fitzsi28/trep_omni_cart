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
MAXSTEP = 295.0 #m/s^2
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"

lqrflag = False

# define initial config and velocity

q0 = np.array([0.,0., 0.]) # x = [x_cart, theta]
dq0 = np.array([0., 0., 0.])

# define time parameters:
t0 = 0.0
tf = 7.7

def build_system():
    sys = trep.System()
    frames = [
        ty('ys', name='y-stylus', kinematic=True),
        ty('yc',name=CARTFRAME, mass=M), [
            rx('theta', name="pendShoulder"), [
                tz(-L, name=MASSFRAME, mass=M)]]]
    sys.import_frames(frames)
    trep.potentials.Gravity(sys, (0,0,-g))
    trep.forces.Damping(sys, B)
    trep.constraints.PointOnPlane(sys, 'y-stylus', (0.,1.0,0.), CARTFRAME)
    return sys

def proj_func(x):
    x[1] = np.fmod(x[1]+np.pi, 2.0*np.pi)
    if(x[1] < 0):
        x[1] = x[1]+2.0*np.pi
    x[1] = x[1] - np.pi

def xdes_func(t, x, xdes):
    xdes[1] = np.pi

def build_sac_control(sys):
    sacsyst = sactrep.Sac(sys)
    sacsyst.T = 0.5
    sacsyst.lam = -20.0
    sacsyst.maxdt = 0.2
    sacsyst.ts = DT
    sacsyst.usat = [[MAXSTEP, -MAXSTEP]]
    sacsyst.calc_tm = DT
    sacsyst.u2search = False
    sacsyst.Q = np.diag([100,200,100,1,40,1]) # yc,th,ys,ycd,thd,ysd
    sacsyst.P = np.diag([0,10,0,0,0,0])
    sacsyst.R = 0.3*np.identity(1)
    sacsyst.set_proj_func(proj_func)
    sacsyst.set_xdes_func(xdes_func)
    return sacsyst

####SETUP#####
system= build_system()
sacsys = build_sac_control(system)
mvi = trep.MidpointVI(system)
mvi.initialize_from_state(t0, q0, np.array([0.,0.]))
sacsys.init()
#########

#def build_LQR_control():
qBar = np.array([0., -np.pi, 0.]) # Desired configuration
Q = np.diag([0.5,1,0.5,0.5,1,0.5]) # Cost weights for states
R = 0.1*np.eye(1) # Cost weights for inputs
TVec = np.arange(t0, tf+DT, DT) # Initialize discrete time vector
dsys = trep.discopt.DSystem(mvi, TVec) # Initialize discrete system
xBar = dsys.build_state(Q=qBar,p = np.zeros(system.nQd)) # Create desired state configuration
Qd = np.zeros((len(TVec), dsys.system.nQ)) # Initialize desired configuration trajectory
thetaIndex = dsys.system.get_config('theta').index # Find index of theta config variable
ycIndex = dsys.system.get_config('yc').index
ysIndex = dsys.system.get_config('ys').index
for i,t in enumerate(TVec):
    Qd[i, thetaIndex] = qBar[1] # Set desired configuration trajectory
    Qd[i, ycIndex] = qBar[0]
    Qd[i, ysIndex] = qBar[2]
    (Xd, Ud) = dsys.build_trajectory(Qd) # Set desired state and input trajectory
Qk = lambda k: Q # Create lambda function for state cost weights
Rk = lambda k: R # Create lambda function for input cost weights
KVec = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk) #Solve for linear controller gain
KStabilize = KVec[0] # use first value to approx infinite-horizon optimal controller gain
dsys.set(np.array([0.,0., 0.,0.,0.,0.]), np.array([0.]), 0)
xTilde = 0.*dsys.xk
#    return 
"""
####SETUP#####
system= build_system()
sacsys = build_sac_control(system)
mvi = trep.MidpointVI(system)
mvi.initialize_from_state(t0, q0, np.array([0.,0.]))
sacsys.init()
#build_LQR_control()
"""

# run loop:
q = np.array((system.q[0], system.dq[0],
               system.q[1], system.dq[1]))
u = np.array([sacsys.controls, sacsys.t_app[1]-sacsys.t_app[0]])
T = [sacsys.time]
Q = [system.q]

while system.t < tf:
    if sacsys.q[1] > -np.pi-0.01 and sacsys.q[1] < -np.pi+0.01:
        lqrflag = True
        mvi.initialize_from_configs(T[-3],Q[-3],T[-2],Q[-2])
        dsys.set(np.array([sacsys.q[0],sacsys.q[1],sacsys.q[2],sacsys.dq[0],sacsys.dq[1],sacsys.dq[2]]),\
                 np.array([sacsys.controls[0]]), round(T[-1]*30))
        
    if lqrflag == True:
        xTilde = dsys.xk-xBar # Compare to desired state
        ulqr = -np.dot(KStabilize, xTilde) # Calculate input
        print ulqr
        dsys.step(ulqr) # Step the system forward by one time step
        print mvi.q1
    else:
        sacsys.step()
    t_app = sacsys.t_app[1]-sacsys.t_app[0]
    xcalc = system.q[0]+(system.dq[0]*t_app) + (0.5*sacsys.controls[0]*t_app*t_app)
        
    fsac = sacsys.controls[0]*M*-1 #SAC resistive force
    q = np.vstack((q, np.hstack((system.q[0], system.q[1],
                                 system.lambda_(), fsac))))
    u = np.vstack((u, np.hstack([sacsys.controls, t_app])))
    T.append(system.t)
    qtemp = system.q
    proj_func(qtemp)
    Q = np.vstack((Q,qtemp))
    if np.abs(system.t%1)<DT:
        print "time = ",system.t
        

plt.plot(T,Q[0:,:-1])
plt.plot(T,u[0:,0])
plt.axis([0,tf,-8,8])
plt.show()    
np.savetxt("x_py.csv", q, fmt="%9.6f", delimiter=",")
np.savetxt("U_py.csv", u, fmt="%9.6f", delimiter=",")

# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])