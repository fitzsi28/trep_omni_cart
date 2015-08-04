import numpy as np
import trep
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import matplotlib.pyplot as plt
import time

# set mass, length, and gravity:
DT = 1./60.
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

# define initial config and velocity

q0 = np.array([3./4.*np.pi,0.]) # x = [theta, xc]
dq0 = np.array([0., 0.])

# define time parameters:
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
    sacsyst.u2search = True#False
    sacsyst.Q = np.diag([200,10,0,0]) # th, x, thd, xd
    sacsyst.P = np.diag([0,0,0,0])
    sacsyst.R = 0.3*np.identity(1)
    sacsyst.set_proj_func(proj_func)
    return sacsyst

system = build_system()
sacsys = build_sac_control(system)

# set initial conditions:
system.q = q0
system.dq = dq0

# init SAC:
sacsys.init()

# run loop:
q = np.array((system.q[0], system.q[1],
               system.dq[0], system.dq[1]))
u = [sacsys.controls[0]]
T = [sacsys.time]
Q = [system.q]

while sacsys.time < tf:
    tic = time.time()
    sacsys.step()
    toc= time.time()
    t_app = sacsys.t_app[1]-sacsys.t_app[0]
    xcalc = system.q[0]+(system.dq[0]*t_app) + (0.5*sacsys.controls[0]*t_app*t_app)
    fsac = sacsys.controls[0]*M*-1 #SAC resistive force
    q = np.vstack((q, np.hstack((system.q[0], system.q[1],
                                 system.dq[0], system.dq[1]))))
    u.append(sacsys.controls[0])
    T.append(sacsys.time)
    qtemp = sacsys.q
    proj_func(qtemp)
    Q = np.vstack((Q,qtemp))
    if np.abs(sacsys.time%1)<DT:
        print "time = ",toc-tic
        
# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])


plt.plot(T,Q)

plt.plot(T,u)
plt.legend(['th','x','u'])
plt.axis([0,tf,-10,10])
plt.show()    
np.savetxt("x_py.csv", q, fmt="%9.6f", delimiter=",")
np.savetxt("U_py.csv", u, fmt="%9.6f", delimiter=",")


