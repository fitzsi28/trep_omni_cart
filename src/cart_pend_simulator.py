#!/usr/bin/env python

"""
Kathleen Fitzsimons
**modified from trep_omni spherical pendulum simulation

This node runs a timer that looks up the TF from the base link of the omni to
the end of the stylus. It then uses the y-portion(right-left) of this pose to 
drive a trep simulation. SAC is used to provide force feedback and the 
direction and size of a green arrow to guide the user.
The position of the cart and pendulum is also published.

SUBSCRIBERS:
    - omni1_button (phantom_omni/PhantomButtonEvent)

PUBLISHERS:
    - mass_point (PointStamped)
    - cart_point (PointStamped)
    - sac_point (PointStamped)
    - visualization_marker_array (MarkerArray)
    - omni1_force_feedback (phantom_omni/OmniFeedback)

SERVICES:

"""

################
# ROS IMPORTS: #
################
import rospy
import tf
from tf import transformations as TR
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg as GM
from phantom_omni.msg import PhantomButtonEvent
from phantom_omni.msg import OmniFeedback
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg as VM


###################
# NON-ROS IMPORTS #
###################
import trep
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import numpy as np
import copy
import time


####################
# GLOBAL CONSTANTS #
####################

DT = 3./100.
M = 0.1 #kg
L = 1 # m
B = 0.01 # damping
g = 9.81 #m/s^2
MAXSTEP = 20 #m/s^2
SACEFFORT=1
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"
SACFRAME = "SAC"
NQ = 3 #number of configuration variables in the system
NU = 1 #number of inputs in the system

def build_system(): #simulated trep system
    system = trep.System()
    frames = [
        ty('ys', name='y-stylus', kinematic=True),
        ty('yc',name=CARTFRAME, mass=M), [
            rx('theta', name="pendShoulder"), [
                tz(-L, name=MASSFRAME, mass=M)]]]
    system.import_frames(frames)
    trep.constraints.PointOnPlane(system, 'y-stylus', (0.,1.0,0.), CARTFRAME)
    trep.potentials.Gravity(system, (0,0,-g))
    trep.forces.Damping(system, B)
    return system


def proj_func(x): #angle wrapping function
    x[1] = np.fmod(x[1]+np.pi, 2.0*np.pi)
    if(x[1] < 0):
        x[1] = x[1]+2.0*np.pi
    x[1] = x[1] - np.pi

def xdes_func(t, x, xdes):
     xdes[1] = np.pi

def build_sac_control(system):
    sacsys=sactrep.Sac(system)
    sacsys.T = 0.5
    sacsys.lam = -20
    sacsys.maxdt = 0.2
    sacsys.ts = DT
    sacsys.usat = [[MAXSTEP, -MAXSTEP]]
    sacsys.calc_tm = DT
    sacsys.u2search = False
    sacsys.Q = np.diag([100,200,100,1,50,1]) # yc,th,ys,ycd,thd,ysd
    sacsys.P = 0*np.diag([0,0,0,0,0,0])
    sacsys.R = 0.3*np.identity(NU)
    sacsys.set_proj_func(proj_func)
    sacsys.set_xdes_func(xdes_func)
    return sacsys

class PendSimulator:

    def __init__(self):
        rospy.loginfo("Creating PendSimulator class")
        
        # define running flag:
        self.running_flag = False
        self.grey_flag = False
        self.fb_flag = False # SAC feedback flag

        # setup markers
        self.setup_markers()
        
        # setup publishers, subscribers, timers:
        self.button_sub = rospy.Subscriber("omni1_button", PhantomButtonEvent, self.buttoncb)
        self.sim_timer = rospy.Timer(rospy.Duration(DT), self.timercb)
        self.mass_pub = rospy.Publisher("mass_point", PointStamped)
        self.cart_pub = rospy.Publisher("cart_point", PointStamped)
        self.sac_pub = rospy.Publisher("sac_point", PointStamped)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray)
        self.force_pub = rospy.Publisher("omni1_force_feedback", OmniFeedback)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        return

    def setup_markers(self):
        self.markers = VM.MarkerArray()
        # mass marker
        self.mass_marker = VM.Marker()
        self.mass_marker.action = VM.Marker.ADD
        self.mass_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
        self.mass_marker.header.frame_id = rospy.get_namespace() + SIMFRAME 
        self.mass_marker.lifetime = rospy.Duration(5*DT)
        self.mass_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.mass_marker.type = VM.Marker.SPHERE
        self.mass_marker.id = 0
        # link marker
        self.link_marker = copy.deepcopy(self.mass_marker)
        self.link_marker.type = VM.Marker.LINE_STRIP
        self.link_marker.color = ColorRGBA(*[0.1, 0.1, 1.0, 1.0])
        self.link_marker.scale = GM.Vector3(*[0.005, 0.05, 0.05])
        self.link_marker.id = 1
        #sac marker
        self.sac_marker = copy.deepcopy(self.mass_marker)
        self.sac_marker.type = VM.Marker.ARROW
        self.sac_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 1.0])
        self.sac_marker.lifetime = rospy.Duration(10*DT)
        self.sac_marker.scale = GM.Vector3(*[0.025, 0.05, 0.025])
        self.sac_marker.id = 2
        #cart marker
        self.cart_marker = copy.deepcopy(self.mass_marker)
        self.cart_marker.type = VM.Marker.CUBE
        self.cart_marker.color = ColorRGBA(*[0.1, 0.5, 1.0, 0.9])
        self.cart_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.cart_marker.id = 3
        
        

        self.markers.markers.append(self.mass_marker)
        self.markers.markers.append(self.sac_marker)
        self.markers.markers.append(self.link_marker)
        self.markers.markers.append(self.cart_marker)
        
        return
    
        
    def setup_integrator(self):
        self.system = build_system()
        self.sacsys = build_sac_control(self.system)
        self.mvi = trep.MidpointVI(self.system)
                    
        # get the position of the omni in the trep frame
        if self.listener.frameExists(SIMFRAME) and self.listener.frameExists(CONTFRAME):
            t = self.listener.getLatestCommonTime(SIMFRAME, CONTFRAME)
            try:
                position, quaternion = self.listener.lookupTransform(SIMFRAME, CONTFRAME, t)
            except (tf.Exception):
                rospy.logerr("Could not transform from "\
                             "{0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
                return
        else:
            rospy.logerr("Could not find required frames "\
                         "for transformation from {0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
            return

        self.q0 = np.array((2*position[1], 0.0, 2*position[1]))
        self.dq0 = np.zeros(self.system.nQd) 
        self.mvi.initialize_from_state(0, self.q0, self.dq0)
        self.system.q = self.mvi.q1
        self.sacsys.init()
        return

    
    def timercb(self, data):
        if not self.running_flag:
            return
        if self.listener.frameExists(SIMFRAME) and self.listener.frameExists(CONTFRAME):
            t = self.listener.getLatestCommonTime(SIMFRAME, CONTFRAME)
            try:
                position, quaternion = self.listener.lookupTransform(SIMFRAME, CONTFRAME, t)
            except (tf.Exception):
                rospy.logerr("Could not transform from "\
                             "{0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
                return
        else:
            rospy.logerr("Could not find required frames "\
                         "for transformation from {0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
            return

        # now we can use this position to integrate the trep simulation:
        ucont = np.zeros(self.mvi.nk)
        ucont[self.system.kin_configs.index(self.system.get_config('ys'))] = 2*position[1]
        
        #compute the SAC control
        self.sacsys.calc_u()
        self.t_app = self.sacsys.t_app[1]-self.sacsys.t_app[0]
#convert kinematic acceleration to new position of SAC marker/change in position amplified by 6
        self.usac = self.system.q[0]+6*((self.system.dq[0]*self.t_app) + (0.5*self.sacsys.controls[0]*self.t_app*self.t_app))
                   
        # step integrator:
        try:
            self.mvi.step(self.mvi.t2 + DT,k2=ucont)
        except trep.ConvergenceError as e:
            rospy.loginfo("Could not take step: %s"%e.message)
            return
              
        # if we successfully integrated, let's publish the point and the tf
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = SIMFRAME
        # get transform from trep world to mass frame:
        gwm = self.system.get_frame(MASSFRAME).g()
        ptrans = gwm[0:3, -1]
        # print ptrans
        p.point.x = ptrans[0]
        p.point.y = ptrans[1]
        p.point.z = ptrans[2]
        self.mass_pub.publish(p)
        # now we can send the transform:
        qtrans = TR.quaternion_from_matrix(gwm)
        self.br.sendTransform(ptrans, qtrans, p.header.stamp, MASSFRAME, SIMFRAME)
        ##cart sim   
 	pc = PointStamped()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = SIMFRAME
        # get transform from trep world to cart frame:
        gwc = self.system.get_frame(CARTFRAME).g()
        ptransc = gwc[0:3, -1]
        # print ptransc
        pc.point.x = ptransc[0]
        pc.point.y = ptransc[1]
        pc.point.z = ptransc[2]
        self.cart_pub.publish(pc)
        # now we can send the transform:
        qtransc = TR.quaternion_from_matrix(gwc)
        self.br.sendTransform(ptransc, qtransc, pc.header.stamp, CARTFRAME, SIMFRAME)
       
        ##sac sim   
 	pu = PointStamped()
        pu.header.stamp = rospy.Time.now()
        pu.header.frame_id = SIMFRAME
        # get transform from trep world to cart frame:
	gwu = copy.copy(gwc)
        gwu.itemset((1,3), self.usac)
	ptransu = gwu[0:3, -1]
        # print ptransc
        pu.point.x = ptransu[0]
        pu.point.y = ptransu[1]
        pu.point.z = ptransu[2]
        self.sac_pub.publish(pu)
        # now we can send the transform:
        qtransu = TR.quaternion_from_matrix(gwu)
        self.br.sendTransform(ptransu, qtransu, pu.header.stamp, SACFRAME, SIMFRAME)
        
        # now we can publish the markers:
        for m in self.markers.markers:
            m.header.stamp = p.header.stamp
        self.mass_marker.pose = GM.Pose(position=GM.Point(*ptrans))
        p1 = GM.Point(*ptrans)
        p2 = GM.Point(*ptransc)
        self.link_marker.points = [p1, p2]
        self.cart_marker.pose = GM.Pose(position=GM.Point(*ptransc))
       
        # now we can render the forces and update the SAC Marker every other iteration:
        if self.fb_flag == False:
            self.render_forces()
            self.sac_marker.points = [GM.Point(*ptransc), GM.Point(*ptransu)]
            self.fb_flag = True
        else:
            self.fb_flag = False

        self.marker_pub.publish(self.markers)
  
        return
        

    def render_forces(self):
        # get the position of the stylus in the omni's base frame
        if self.listener.frameExists(BASEFRAME) and self.listener.frameExists(CONTFRAME):
            t = self.listener.getLatestCommonTime(BASEFRAME, CONTFRAME)
            try:
                position, quaternion = self.listener.lookupTransform(BASEFRAME, CONTFRAME, t)
            except (tf.Exception):
                rospy.logerr("Could not transform from "\
                             "{0:s} to {1:s}".format(BASEFRAME,CONTFRAME))
                return
        else:
            rospy.logerr("Could not find required frames "\
                         "for transformation from {0:s} to {1:s}".format(BASEFRAME,CONTFRAME))
            return
        # get force magnitude
        fsac = np.array([0.,(SACEFFORT*self.sacsys.controls[0]*self.t_app),0.])
        # the following transform was figured out only through
        # experimentation. The frame that forces are rendered in is not aligned
        # with /trep_world or /base:
        fvec = np.array([fsac[1], fsac[2], fsac[0]])
        f = GM.Vector3(*fvec)
        p = GM.Vector3(*position)
        self.force_pub.publish(OmniFeedback(force=f, position=p))
        return
        
    def buttoncb(self, data):
        if data.grey_button == 1 and data.white_button == 0 and self.running_flag == False:
            rospy.loginfo("Integration primed")
            self.grey_flag = True
        elif data.grey_button == 0 and data.white_button == 0 and self.grey_flag == True and self.running_flag == False:
            # then we previously pushed only the grey button, and we just released it
            rospy.loginfo("Starting integration")
            self.setup_integrator()
            self.running_flag = True
        elif data.grey_button == 0 and data.white_button == 0 and self.grey_flag == True and self.running_flag == True:
            # then the sim is already running and nothing should happend
            rospy.loginfo("Integration already running")
        elif data.white_button == 1:
            rospy.loginfo("Integration stopped")
            self.grey_flag = False
            self.running_flag = False
            self.force_pub.publish(OmniFeedback(force=GM.Vector3(), position=GM.Vector3()))
        return



def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('cart_pend_sim', log_level=rospy.INFO)

    try:
        sim = PendSimulator()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
