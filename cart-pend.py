import numpy as np
import trep
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import matplotlib.pyplot as plt
import time

# set mass, length, and gravity:
DT = 1./60.
M = 0.2 #kg
L = 1.0 # m
B = 0.01 # damping
g = 9.81 #m/s^2
MAXSTEP = 35.0 #m/s^2
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"

# define initial config and velocity

q0 = np.array([0., 0.]) # x = [theta, x_cart]
dq0 = np.array([0., 0.])

# define time parameters:
#dt = 0.0167
tf = 10.0

# create system
system = trep.System()
# define frames
frames = [
    ty('ys', name='y-stylus', kinematic=True), [
        rx('theta', name="pendShoulder"), [
            tz(-L, name=MASSFRAME, mass=M)]]]
# add frames to system
system.import_frames(frames)
# add gravity potential
trep.potentials.Gravity(system, (0,0,-g))
# add a damping force on the cart
trep.forces.Damping(system, B)

#add a constraint
#trep.constraints.PointOnPlane(system, 'y-stylus', (0.,1.0,0.), CARTFRAME)


#############
# SAC STUFF #
#############

def proj_func(x):
    x[0] = np.fmod(x[0]+np.pi, 2.0*np.pi)
    if(x[0] < 0):
        x[0] = x[0]+2.0*np.pi
    x[0] = x[0] - np.pi

def xdes_func(t, x, xdes):
    xdes[0] = np.pi

sacsys = sactrep.Sac(system)

sacsys.T = 1.2
sacsys.lam = -5
sacsys.maxdt = 0.2
sacsys.ts = DT
sacsys.usat = [[MAXSTEP, -MAXSTEP]]
sacsys.calc_tm = 0#DT
sacsys.u2search = False
sacsys.Q = np.diag([200,50,0,0]) # th,ys,thd,ysd
sacsys.P = 0*np.diag([0,0,0,0])
sacsys.R = 0.3*np.identity(1)

sacsys.set_proj_func(proj_func)
sacsys.set_xdes_func(xdes_func)

# set initial conditions:
system.q = q0
system.dq = dq0

# init SAC:
sacsys.init()

# run loop:
q = np.array((system.q[0], system.q[1],
               system.dq[0], system.dq[1]))
u = np.array([sacsys.controls, sacsys.t_app[1]-sacsys.t_app[0]])
T = [sacsys.time]
Q = [system.q]

while sacsys.time < tf:
    #sacsys.Q = np.diag([np.power(system.q[0]/0.5,8),200,np.power(system.q[2]/0.5,8),0,50,0])
    tic = time.time()
    sacsys.step()
    toc = time.time()
    t_app = sacsys.t_app[1]-sacsys.t_app[0]
    xcalc = system.q[0]+(system.dq[0]*t_app) + (0.5*sacsys.controls[0]*t_app*t_app)
    q = np.vstack((q, np.hstack((system.q[0], system.dq[1],
                                 t_app, sacsys.controls[0]))))
    u = np.vstack((u, np.hstack([sacsys.controls, t_app])))
    T.append(sacsys.time)
    qtemp = sacsys.q
    proj_func(qtemp)
    Q = np.vstack((Q,qtemp))
    if np.abs(sacsys.time%1)<DT:
        print "time = ",(toc-tic)
        
plt.plot(T,Q)
plt.plot(T,u[0:])
plt.legend(["Theta","y","U"])
plt.show()    
np.savetxt("x_py.csv", q, fmt="%9.6f", delimiter=",")
np.savetxt("U_py.csv", u, fmt="%9.6f", delimiter=",")

# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])

