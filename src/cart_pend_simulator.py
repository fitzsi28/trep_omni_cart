#!/usr/bin/env python

"""
Kathleen Fitzsimons
**modified from trep_omni spherical pendulum simulation

This node runs 3 timers. (1) that looks up the TF from the base link of the omni to
the end of the stylus. It then uses the y-portion(right-left) of this pose to 
drive a trep simulation. (2) SAC timer computes the optimal action for the current state and uses it to update the location and directionality of a virtual 'wall'. (3) updates the forces to render the virtual wall and calculate the SAC score.
The position of the cart and pendulum is also published.

SUBSCRIBERS:
    - omni1_button (phantom_omni/PhantomButtonEvent)

PUBLISHERS:
    - mass_point (PointStamped)
    - cart_point (PointStamped)
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
import trep.discopt
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import numpy as np
from numpy import dot
import copy
import time


####################
# GLOBAL CONSTANTS #
####################

DT = 1./60.
TS = 1./1.
M = 0.1 #kg
L = 1 # m
B = 0.1 # damping
g = 9.81 #m/s^2
SCALE = 8
Kp = 200.0/SCALE
Kd = 50.0/SCALE
WALL = SCALE*0.2
MAXSTEP = 20. #m/s^2
SACEFFORT=1.0*SCALE
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"
SACFRAME = "SAC"
NQ = 2 #number of configuration variables in the system
NU = 1 #number of inputs in the system

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

class PendSimulator:

    def __init__(self):
        rospy.loginfo("Creating PendSimulator class")
        
        # define running flag:
        self.running_flag = False
        self.grey_flag = False
        self.upos = 0.
        self.vel = 0.
        self.prev = np.array([0.,0.,0.])
        self.wall=0.
        self.i = 0.
        self.n = 0.
        
        # setup markers
        self.setup_markers()
        
        # setup publishers, subscribers, timers:
        self.button_sub = rospy.Subscriber("omni1_button", PhantomButtonEvent, self.buttoncb)
        self.sim_timer = rospy.Timer(rospy.Duration(DT), self.timercb)
        self.sac_timer = rospy.Timer(rospy.Duration(TS), self.timersac)
        self.force_timer = rospy.Timer(rospy.Duration(DT),self.render_forces)
        self.mass_pub = rospy.Publisher("mass_point", PointStamped, queue_size = 1)
        self.cart_pub = rospy.Publisher("cart_point", PointStamped, queue_size = 1)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray, queue_size = 1)
        self.force_pub = rospy.Publisher("omni1_force_feedback", OmniFeedback , queue_size = 1)
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
        self.mass_marker.lifetime = rospy.Duration(4*DT)
        self.mass_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.mass_marker.type = VM.Marker.SPHERE
        self.mass_marker.id = 0
        # link marker
        self.link_marker = copy.deepcopy(self.mass_marker)
        self.link_marker.type = VM.Marker.LINE_STRIP
        self.link_marker.color = ColorRGBA(*[0.1, 0.1, 1.0, 1.0])
        self.link_marker.scale = GM.Vector3(*[0.005, 0.05, 0.05])
        self.link_marker.id = 1
        #cart marker
        self.cart_marker = copy.deepcopy(self.mass_marker)
        self.cart_marker.type = VM.Marker.CUBE
        self.cart_marker.color = ColorRGBA(*[0.1, 0.5, 1.0, 0.9])
        self.cart_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.cart_marker.id = 3
        #sac marker
        self.sac_marker = copy.deepcopy(self.cart_marker)
        self.sac_marker.type = VM.Marker.LINE_STRIP
        self.sac_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 1.0])
        self.sac_marker.lifetime = rospy.Duration(3*DT)
        self.sac_marker.scale = GM.Vector3(*[0.015, 0.015, 0.015])
        p1 = np.array([0.0,0.0,0.1])
        p2 = np.array([0.0,0.075,0.2])
        p3 = np.array([0.0,-0.05,0.15])
        self.sac_marker.points = [GM.Point(*p3), GM.Point(*p1), GM.Point(*p2)]
        self.sac_marker.id = 2
        # score marker
        self.score_marker = copy.deepcopy(self.mass_marker)
        self.score_marker.type = VM.Marker.TEXT_VIEW_FACING
        self.score_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
        self.score_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.score_marker.pose.position.x = 0;
        self.score_marker.pose.position.y = 0;
        self.score_marker.pose.position.z = 0.2;
        self.score_marker.pose.orientation.x = 0.0;
        self.score_marker.pose.orientation.y = 0.0;
        self.score_marker.pose.orientation.z = 0.0;
        self.score_marker.pose.orientation.w = 1.0;
        self.score_marker.text = "0%"
        self.score_marker.id = 4
        
        self.markers.markers.append(self.mass_marker)
        self.markers.markers.append(self.link_marker)
        self.markers.markers.append(self.cart_marker)
        self.markers.markers.append(self.sac_marker)
        self.markers.markers.append(self.score_marker)

        return
    
        
    def setup_integrator(self):
        self.system = build_system()
        self.lqrsys = build_system()
        self.lqrmvi = trep.MidpointVI(self.lqrsys)
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

        self.q0 = np.array([0.0, SCALE*position[1]])#X=[th,yc]
        self.dq0 = np.zeros(self.system.nQd) 
        self.mvi.initialize_from_state(0, self.q0, self.dq0)
        self.lqrmvi.initialize_from_state(0, self.q0, self.dq0)
        self.lqrsys.q = self.system.q
        self.lqrsys.dq = self.system.dq
        X0 = np.array([self.system.q[0],self.system.q[1],self.system.dq[0],self.system.dq[1]])
        self.dsys, self.xBar,self.KStabil = build_lqr_control(self.lqrsys,self.lqrmvi, X0)
        #compute the LQR control
        x = self.dsys.xk # Grab current state
        xTilde = x - self.xBar # Compare to desired state
        self.u = -dot(self.KStabil, xTilde) # Calculate input
        #dsys.step(u) # Step the system forward by one time step
        
        #convert kinematic acceleration to new velocity&position
        self.uvel = self.system.dq[1]+self.u*DT
        self.upos = self.system.q[1] +0.5*(self.uvel+self.system.dq[1])*DT        
        self.wall = SCALE*position[1]
        #reset score values
        self.i = 0.
        self.n = 0.
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

        #update position array
        self.prev = np.insert(self.prev,0, SCALE*position[1])
        self.prev = np.delete(self.prev, -1)
        # now we can use this position to integrate the trep simulation:
        ucont = np.zeros(self.mvi.nk)
        ucont[self.system.kin_configs.index(self.system.get_config('yc'))] = self.prev[0]
        
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
        gwc = self.system.get_frame(CARTFRAME).g()
        ptransc = gwc[0:3, -1]
        pc.point.x = ptransc[0]
        pc.point.y = ptransc[1]
        pc.point.z = ptransc[2]
        self.cart_pub.publish(pc)
        qtransc = TR.quaternion_from_matrix(gwc)
        self.br.sendTransform(ptransc, qtransc, pc.header.stamp, CARTFRAME, SIMFRAME)
        
        # now we can publish the markers:
        for m in self.markers.markers:
            m.header.stamp = p.header.stamp
        self.mass_marker.pose = GM.Pose(position=GM.Point(*ptrans))
        p1 = GM.Point(*ptrans)
        p2 = GM.Point(*ptransc)
        self.link_marker.points = [p1, p2]
        self.cart_marker.pose = GM.Pose(position=GM.Point(*ptransc))
        self.marker_pub.publish(self.markers)
        return
        

    def timersac(self,data):
        if not self.running_flag:
            return
        #compute the LQR control
        x = self.dsys.xk # Grab current state
        xTilde = x - self.xBar # Compare to desired state
        self.u = -dot(self.KStabil, xTilde) # Calculate input
        #dsys.step(u) # Step the system forward by one time step
        
        #convert kinematic acceleration to new velocity&position
        veltemp = self.system.dq[1]+self.u*DT
        self.upos = self.system.q[1] +0.5*(self.uvel+self.system.dq[1])*DT        
        if np.sign(self.uvel) != np.sign(veltemp):#update wall if sac changes direction
            self.wall = self.prev[0]
        self.uvel = veltemp
        return
    
    def render_forces(self,data):
        if not self.running_flag:
            return
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
        #get force magnitude
        fsac = np.array([0.,0.,0.])
        if (self.uvel > 0 and SCALE*position[1] < self.wall) or \
           (self.uvel < 0 and SCALE*position[1] > self.wall):
            fsac = np.array([0.,Kp*(self.wall-SCALE*position[1]) \
                             +Kd*(self.prev[1]-self.prev[0]),0.])
            self.sac_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 0.0])
        elif abs(SCALE*position[1] - self.prev[1]) < SCALE*10**(-4) and self.uvel == 0.0:
            self.sac_marker.color = ColorRGBA(*[0.05, 0.05, 1.0, 1.0])
            self.i += 1
        elif abs(SCALE*position[1] - self.prev[1]) < SCALE*10**(-4):
            self.sac_marker.color = ColorRGBA(*[0.05, 0.05, 1.0, 0.0])
        else:
            self.sac_marker.color = ColorRGBA(*[0.05, 1.0, 0.05, 1.0]) 
            self.i += 1 
        self.n += 1
        self.score_marker.text = "Score = "+ str(round((self.i/self.n)*100,2))+"%"
        # the following transform was figured out only through
        # experimentation. The frame that forces are rendered in is not aligned
        # with /trep_world or /base:
        fvec = np.array([fsac[1], fsac[2], fsac[0]])
        f = GM.Vector3(*fvec)
        p = GM.Vector3(*position)
        self.force_pub.publish(OmniFeedback(force=f, position=p))
        return
           
    def buttoncb(self, data):
        if data.grey_button == 1 and data.white_button == 0 and \
        self.running_flag == False:
            rospy.loginfo("Integration primed")
            self.grey_flag = True
        elif data.grey_button == 0 and data.white_button == 0 and \
        self.grey_flag == True and self.running_flag == False:
            # then we previously pushed only the grey button, and we just released it
            rospy.loginfo("Starting integration")
            self.setup_integrator()
            self.running_flag = True
        elif data.grey_button == 0 and data.white_button == 0 and \
        self.grey_flag == True and self.running_flag == True:
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
