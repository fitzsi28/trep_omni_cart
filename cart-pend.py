import numpy as np
import trep
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import matplotlib.pyplot as plt

# set mass, length, and gravity:
DT = 4./100.
M = 0.05 #kg
L = 0.5 # m
B = 0.002 # damping
g = 9.81 #m/s^2
MAXSTEP =10.0 #m
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"

# define initial config and velocity
q0 = np.array([0, np.pi-0.01]) # x = [x_cart, theta]
dq0 = np.array([0, 0])

# define time parameters:
#dt = 0.0167
tf = 15.0

# create system
system = trep.System()
# define frames
frames = [
    ty('yc',name=CARTFRAME, mass=M), [
        rx('theta', name="pendShoulder"), [
            tz(-L, name=MASSFRAME, mass=M)]]]
# add frames to system
system.import_frames(frames)
# add gravity potential
trep.potentials.Gravity(system, (0,0,-g))
# add a damping force on the cart
trep.forces.Damping(system, B)
trep.forces.ConfigForce(system,"yc","cart_force")


#############
# SAC STUFF #
#############

def proj_func(x):
    x[1] = np.fmod(x[1]+np.pi, 2.0*np.pi)
    if(x[1] < 0):
        x[1] = x[1]+2.0*np.pi
    x[1] = x[1] - np.pi

def xdes_func(t, x, xdes):
    xdes[1] = np.pi

sacsys = sactrep.Sac(system)

sacsys.T = 1.2
sacsys.lam = -5 #gamma
sacsys.maxdt = 0.2
sacsys.ts = DT
sacsys.usat = [[MAXSTEP, -MAXSTEP]]
sacsys.calc_tm = DT
sacsys.u2search = False
sacsys.Q = np.diag([100,200,50,0]) # x,th,xd,thd
sacsys.P = 0*np.diag([0,0,0,0])
sacsys.R = 0.3*np.identity(1)

sacsys.set_proj_func(proj_func)
sacsys.set_xdes_func(xdes_func)

# set initial conditions:
system.q = q0
system.dq = dq0
T = [sacsys.time]
Q = [system.q]
# init SAC:
sacsys.init()

# run loop:
q = np.array((system.q[0], system.dq[0],
               system.q[1], system.dq[1]))
u = np.array([sacsys.controls])
T = [sacsys.time]
Q = [sacsys.q]
#T.append(sacsys.time)
#Q.append(system.q)
while sacsys.time < tf:
    sacsys.Q = np.diag([np.power(system.q[0]/0.5,8),200,50,0])
    sacsys.step()
    q = np.vstack((q, np.hstack((system.q[0], system.q[1],
                                 system.dq[0],system.dq[1]))))
    u = np.vstack((u, sacsys.controls))
    T.append(sacsys.time)
    qtemp = sacsys.q
    proj_func(qtemp)
    Q.append(qtemp)
    if np.abs(sacsys.time%1)<DT:
        print "time = ",sacsys.time
        
plt.plot(T,Q)
plt.plot(T,u)
plt.show()    
np.savetxt("x_py.csv", q, fmt="%9.6f", delimiter=",")
np.savetxt("U_py.csv", u, fmt="%9.6f", delimiter=",")

# Visualize the system in action
trep.visual.visualize_3d([ trep.visual.VisualItem3D(system, T, Q) ])

