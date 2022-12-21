'''
play_basketball2.py
   Kinematics for a robot to shoot basketballs into a hoop.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState
                markers (ball, hoop)
                
TODO Ideas:
   - Add wrist and have ball obey physics always
   - Plots of release points (score of how good is a release point)
'''

import rclpy
import numpy as np
import math

from sensor_msgs.msg    import JointState
from rclpy.node         import Node

###################
# Import helpers
from KinematicChain    import KinematicChain
from hw5code.TransformHelpers  import *
from math import (pi, sin, cos, asin, acos, atan2)

# Import packages to make basketball and hoop markers
from rclpy.qos              import QoSProfile, DurabilityPolicy
from geometry_msgs.msg      import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

# Import packages for keyboard input
import sys
import select
###################

# Interactive Hoop
class Hoop:
    def __init__(self, node, position, radius):
        # Store the reference to the node.
        self.node  = node

        # Instantiate the point p (as a numpy vector).
        self.p = np.array(position).reshape((3,1))

        # Create a marker of type LINE_STRIP for the ring.
        marker = Marker()
        marker.header.frame_id    = "world"
        marker.header.stamp       = node.get_clock().now().to_msg()
        marker.ns                 = "ring"
        marker.id                 = 0
        marker.type               = Marker.LINE_STRIP
        marker.action             = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.position.x    = position[0] # Center
        marker.pose.position.y    = position[1]
        marker.pose.position.z    = position[2]
        marker.scale.x            = 0.2*radius  # Linewidth
        # marker.scale.y            = 1.0       # Ignored
        # marker.scale.z            = 1.0
        marker.color.r            = 238 / 256         
        marker.color.g            = 103 / 256
        marker.color.b            = 48 / 256
        marker.color.a            = 1.0         # Make solid

        # Add the list of points to make the ring around the center.
        N = 32
        for i in range(N+1):
            theta = 2 * pi *  float(i) / float(N)
            marker.points.append(Point())
            marker.points[-1].x = radius * cos(theta)
            marker.points[-1].y = radius * sin(theta)
            marker.points[-1].z = 0.0

        # Create an interactive marker for our server.  Set the
        # position to coincide with the ring, and scale so the handles
        # reach outside the ring.
        imarker = InteractiveMarker()
        imarker.header.frame_id = "world"
        imarker.name = "ring"
        imarker.pose.position.x = position[0]
        imarker.pose.position.y = position[1]
        imarker.pose.position.z = position[2]
        imarker.scale = 2.0 * radius

        # Append a non-interactive control which contains the ring.
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        imarker.controls.append(control)

        # Append an interactive control for X movement
        # The x axis is the main move-axis.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)
        
        # Append an interactive control for Y movement
        # Rotate the main move-axis (x) by 90 deg about z.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = sin(0.25*pi)
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)

        # Append an interactive control for Z movement.
        # Rotate the main move-axis (x) by 90 deg about y.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = sin(0.25*pi)
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)
        
        # Create an interactive marker server on the topic namespace
        # ring, setup the feedback, and apply/send to all clients.
        server = InteractiveMarkerServer(node, 'ring')
        server.insert(imarker, feedback_callback=self.process)
        server.applyChanges()

        # Report.
        node.get_logger().info("Interactive Hoop set up...")

    def position(self):
        # Return the point.
        return self.p

    def process(self, msg):
        # Save the point contained in the message.
        self.p[0,0] = msg.pose.position.x
        self.p[1,0] = msg.pose.position.y
        self.p[2,0] = msg.pose.position.z

        # Report (to show working)
        self.node.get_logger().info("Hoop moved to: " + str(self.p.T))
      
# Interactive Release Point
class ReleasePoint:
    def __init__(self, node, position, radius):
        # Store the reference to the node.
        self.node  = node

        # Instantiate the point p (as a numpy vector).
        self.p = np.array(position).reshape((3,1))
        
        # Create the point marker.
        marker = Marker()
        marker.header.frame_id    = "world"
        marker.header.stamp       = node.get_clock().now().to_msg()
        marker.ns                 = "release_point"
        marker.id                 = 2
        marker.type               = Marker.SPHERE
        marker.action             = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.position.x    = position[0] # Center
        marker.pose.position.y    = position[1]
        marker.pose.position.z    = position[2]
        marker.scale.x            = 2 * radius  
        marker.scale.y            = 2 * radius
        marker.scale.z            = 2 * radius
        marker.color.r            = 0.0         
        marker.color.g            = 1.0
        marker.color.b            = 0.0
        marker.color.a            = 1.0         # Make solid

        # Create an interactive marker for our server.  Set the
        # position to coincide with the point.
        imarker = InteractiveMarker()
        imarker.header.frame_id = "world"
        imarker.name = "release_point"
        imarker.pose.position.x = position[0]
        imarker.pose.position.y = position[1]
        imarker.pose.position.z = position[2]
        imarker.scale = 2.0 * radius

        # Append a non-interactive control which contains the point.
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        imarker.controls.append(control)

        # Append an interactive control for X movement
        # The x axis is the main move-axis.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)
        
        # Append an interactive control for Y movement
        # Rotate the main move-axis (x) by 90 deg about z.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = sin(0.25*pi)
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)

        # Append an interactive control for Z movement.
        # Rotate the main move-axis (x) by 90 deg about y.
        # Fixed means the control axis remains fixed with the world frame.
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.orientation.w = cos(0.25*pi)
        control.orientation.x = 0.0
        control.orientation.y = sin(0.25*pi)
        control.orientation.z = 0.0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        imarker.controls.append(control)
        
        # Create an interactive marker server on the topic namespace
        # point, setup the feedback, and apply/send to all clients.
        server = InteractiveMarkerServer(node, 'release_point')
        server.insert(imarker, feedback_callback=self.process)
        server.applyChanges()

        # Report.
        node.get_logger().info("Interactive Release Point set up...")

    def position(self):
        # Return the point.
        return self.p

    def process(self, msg):
        # Save the point contained in the message.
        self.p[0,0] = msg.pose.position.x
        self.p[1,0] = msg.pose.position.y
        self.p[2,0] = msg.pose.position.z

        # Report (to show working)
        self.node.get_logger().info("Hoop moved to: " + str(self.p.T))


'''
Returns the position and velocity for a cubic spline with these parameters:
    p0, pf: initial, final position
    v0, vf: initial, final velocity
         t: time since start of motion
         T: total time for motion
'''
def spline(p0, pf, v0, vf, t, T):
    a = p0
    b = v0
    c = 3 * (pf - p0) / T**2 - vf / T - 2 * v0 / T
    d = -2 * (pf - p0) / T**3 + vf / T**2 + v0 / T**2
    return a + b * t + c * t**2 + d * t**3, b + 2 * c * t + 3 * d * t**2
    
   
'''
Returns an array that at each index i has the middle value of
min_vec[i], vec[i], and max_vec[i]
''' 
def assert_vec_bounds(vec, min_vec, max_vec):
    arr = [max(min_vec[i], min(max_vec[i], vec[i])) for i in range(len(vec))]
    return np.array(arr).reshape(-1, 1)
    
'''
Returns the smallest magnitude velocity vector to hit the hoop from the release point
'''
def get_release_velocity(p_hoop, p_release, t):
    g = np.array([0.0, 0.0, -9.81]).reshape((-1,1))
    return (p_hoop - p_release - 0.5 * g * t ** 2) / t


#   Trajectory Class
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Initialize Chain
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        
        # Initial Kinematics
        self.q0 = np.array([ 0.0, pi/2,  0.0, 0.0]).reshape((-1,1))
        self.q = self.q0
        self.qdot = np.array([ 0.0, 0.0,  0.0, 0.0]).reshape((-1,1))
        self.chain.setjoints(self.q)
        self.lam = 10
        
        # Important Task Space Positions
        self.p0 = self.chain.ptip()
        self.v0 = np.array([0.0, 0.0, 0.0]).reshape((-1,1))

        
        self.p_ball_pile = np.array([1.0, 1.0, 0.15]).reshape((-1,1))
        self.v_ball_pile = np.array([0.0, 0.0, 0.0]).reshape((-1,1))
        
        # Important Joint Space Positions
        self.q0 = self.tasktojoint(self.p0)
        J = self.Jv(self.q0)
        try:
            self.qdot0 = (np.transpose(J) @ np.linalg.inv(J @ np.transpose(J))) @ self.v0
        except:
            print("Reached a singularity. Try a different configuration!")
        
        self.q_ball_pile = self.tasktojoint(self.p_ball_pile)
        J = self.Jv(self.q_ball_pile)
        try:
            self.qdot_ball_pile = (np.transpose(J) @ np.linalg.inv(J @ np.transpose(J))) @ self.v_ball_pile
        except:
            print("Reached a singularity. Try a different configuration!")
        
        # State Variables
        self.wait = True
        self.holding_ball = True
        
        self.currp = self.chain.ptip()
        self.currv = self.v0
        self.release_set = False
        
    # Forward kinematics
    def fkin(self, q):
        return np.array([-np.sin(q[0]) * ((1 + q[2])*np.cos(q[1]) + np.cos(q[1] + q[3])),
                          np.cos(q[0]) * ((1 + q[2])*np.cos(q[1]) + np.cos(q[1] + q[3])),
                         (1 + q[2])*np.sin(q[1]) + np.sin(q[1] + q[3])])
                         
    # Jacobian of q                     
    def Jv(self, q):
        J = np.zeros((3,len(self.jointnames())))
        J[0, 0] = -np.cos(q[0])*((1 + q[2])*np.cos(q[1]) + np.cos(q[1] + q[3]))
        J[1, 0] = -np.sin(q[0])*((1 + q[2])*np.cos(q[1]) + np.cos(q[1] + q[3]))
        J[2, 0] = 0
        
        J[0, 1] = np.sin(q[0]) * ((1 + q[2])*np.sin(q[1]) + np.sin(q[1] + q[3]))
        J[1, 1] = -np.cos(q[0]) * ((1 + q[2])*np.sin(q[1]) + np.sin(q[1] + q[3]))
        J[2, 1] = (1 + q[2])*np.cos(q[1]) + np.cos(q[1] + q[3])
        
        
        J[0, 2] = -np.sin(q[0]) * np.cos(q[1])
        J[1, 2] = np.cos(q[0]) * np.cos(q[1])
        J[2, 2] = np.sin(q[1])
        
        
        J[0, 3] = np.sin(q[0]) * np.sin(q[1] + q[3])
        J[1, 3] = -np.cos(q[0]) * np.sin(q[1] + q[3])
        J[2, 3] = np.cos(q[1] + q[3])
        
        return J
    
    # Newton Raphson iterative process
    def tasktojoint(self, x):
        prevq = self.q0
        err = ep(x, self.fkin(prevq))
        count = 0
        while np.linalg.norm(err) >= .01:
        
            if count == 10000:
                prevq[0] = -69
                return prevq
            J = self.Jv(prevq)
            J_inv = np.linalg.pinv(J)
            newq = prevq + J_inv @ err
            prevq = newq
            prevq[0] = math.remainder(prevq[0], np.pi)
            prevq[1] = math.remainder(prevq[1], np.pi)
            prevq[3] = math.remainder(prevq[3], np.pi)
            if prevq[2] < 0:
                prevq[2] = 0
            err = ep(x, self.fkin(prevq))
            count += 1

        return prevq
    

    # Declare the joint names.
    def jointnames(self):
        return ['theta1', 'theta2', 'd1', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        if self.wait:
            # Waiting to start throwing motion
            self.ball_state = 'ground'
            pd, vd = self.q0, self.qdot0
        elif t < 3:
            #print(self.p_ball_pile)
            #print(self.fkin(self.q_ball_pile))
            # print(self.q_ball_pile)
            # print()
            # Going to pick up ball
            if self.q_ball_pile[0] == -69:
                print("Basketbro cannot reach ball pile! Please change configuration!")
                self.ball_state = 'ground'
                self.wait = True
                self.q, self.qdot0 = self.q0, self.qdot0
                self.chain.setjoints(self.q)
                return (
                    self.q.flatten().tolist(),
                    self.qdot.flatten().tolist(),
                    self.chain.ptip().flatten().tolist(),
                    self.v0.flatten().tolist()
                    )
                
            pd, vd = spline( self.q, self.q_ball_pile, self.qdot, self.qdot_ball_pile, dt, max(3 - t, 0.01))
        elif t >= 3 and t < 6:
            # Throwing motion
            t1 = t - 3
            self.ball_state = 'holding'
            
            if self.q_release[0] == -69:
                print("Basketbro cannot reach release point! Please change configuration!")
                self.ball_state = 'ground'
                self.wait = True
                self.q, self.qdot0 = self.q0, self.qdot0
                self.chain.setjoints(self.q)
                return (
                    self.q.flatten().tolist(),
                    self.qdot.flatten().tolist(),
                    self.chain.ptip().flatten().tolist(),
                    self.v0.flatten().tolist()
                    )
            elif self.release_set == False:
                J = self.Jv(self.q_release)
                self.qdot_release = (np.transpose(J) @ np.linalg.inv(J @ np.transpose(J))) @ self.v_release
                self.release_set = True
                
            pd, vd = spline( self.q, self.q_release, self.qdot, self.qdot_release, dt, max(0.01, 3 - t1))
        elif t >= 6 and t < 9:
            self.release_set = False
            t1 = t - 6
            # Smoothly slowing down and going back to start configuration
            self.ball_state = 'free'
            pd, vd = spline( self.q, self.q0, self.qdot, self.qdot0, dt, max(0.01, 3 - t1))
        else:
            # The end of the throwing motion. Wait at start configuration and reset state variables.
            self.ball_state = 'ground'
            self.wait = True
            pd, vd = self.q0, self.qdot0
            
        # print(str(t) + " " + str(self.ball_state))
        
        
        # INVERSE KINEMATICS
        # Compute Jacobian
        J = self.chain.Jv()
        # print(J)
        # print(self.Jv(self.q))
        # J_inv = np.linalg.pinv(J)
        try:
            J_inv = np.transpose(J) @ np.linalg.inv(J @ np.transpose(J))
                    
            # Find qdot
            errP = ep(pd, self.q)
            # self.qdot = np.matmul(J_inv, vd + self.lam * errP)
            self.qdot = vd 
            # self.qdot += (np.eye(len(J_inv)) - J_inv @ J) @ (0.1 * (self.q0 - self.q))
        
            self.qdot = assert_vec_bounds(self.qdot.flatten().tolist(), [-2.0, -3.0, -3.0, -5.0], [2.0, 3.0, 3.0, 5.0])
            
            
          
            # Integrate the joint position, update, store as appropriate.
            self.q = self.q + dt * self.qdot
            self.q = assert_vec_bounds(self.q.flatten().tolist(), [-np.pi, -np.pi, 0.0, -pi], [pi, pi, 3.0, pi])
        
            # Update chain
            self.chain.setjoints(self.q)
        
            # self.currp = self.chain.ptip()
            # self.currv = vd
        except:
            print("Reached a singularity. Try a different configuration!")
            self.ball_state = 'ground'
            self.wait = True
            self.q, self.qdot0 = self.q0, self.qdot0
            self.chain.setjoints(self.q)


        # Return the joint position and velocity, and the tip position and velocity for the ball to render
        return (
            self.q.flatten().tolist(),
            self.qdot.flatten().tolist(),
            self.chain.ptip().flatten().tolist(),
            (J @ self.qdot).flatten().tolist()
        )

#   Generator Node Class
class Generator(Node):
    # Initialization.
    def __init__(self):
        # Initialize the node, naming it 'generator'
        super().__init__('generator')

        # Initialize list of publishers with one to send the joint commands.
        self.pubs = [self.create_publisher(JointState, '/joint_states', 10)]

        # Wait for a connection to happen
        self.get_logger().info("Waiting for a subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Set up a trajectory.
        self.trajectory = Trajectory(self)
        self.jointnames = self.trajectory.jointnames()

        # Create a timer to keep calculating/sending commands.
        self.starttime = self.get_clock().now()
        rate           = 100
        self.timer     = self.create_timer(1/float(rate), self.update)
        
        self.t         = 0.0
        self.dt        = self.timer.timer_period_ns * 1e-9
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))
                               
        ###################
        # Create hoop and release point
        self.hoop = Hoop(self, [-3.0, 3.0, 1.0], 0.2)
        self.release_point = ReleasePoint(self, [-0.5, 1.0, 2.9], 0.05)
        
        # Create marker for basketball
        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pubs.append(self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality))

        # Initialize the ball position and velocity.
        self.R    = 0.15
        self.p    = Point()
        self.v    = Point()

        # Create the sphere marker.
        self.marker = Marker()
        self.marker.header.frame_id    = "world"
        self.marker.header.stamp       = self.get_clock().now().to_msg()

        self.marker.action             = Marker.ADD
        self.marker.ns                 = "point"
        self.marker.id                 = 1
        self.marker.type               = Marker.SPHERE
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position      = self.p
        self.marker.scale.x            = 2*self.R
        self.marker.scale.y            = 2*self.R
        self.marker.scale.z            = 2*self.R
        self.marker.color.r            = 238 / 256
        self.marker.color.g            = 103 / 256
        self.marker.color.b            = 48 / 256
        self.marker.color.a            = 1.0         # Make solid

        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)
        ###################
    
    # Returns true if the ball is currently passing through the hoop, else false
    def ball_in_hoop(self):
        ball_near = abs(self.p.x - self.hoop.p[0][0]) < 0.2 and abs(self.p.y - self.hoop.p[1][0]) < 0.2
        return ball_near and self.p.z <= self.hoop.p[2][0] + self.R

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()

    # Update - send a new joint command every time step.
    def update(self):
        # Grab the current time.
        now = self.get_clock().now()
        t   = (now - self.starttime).nanoseconds * 1e-9
        
        # To avoid the time jitter introduced by an inconsistent timer,
        # just enforce a constant time step and ignore the above.
        self.t += self.dt
        
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            # Grab the input so it clears
            user_input = input('')
            
            # Compute new release point and velocity
            self.trajectory.p_release = np.array(self.release_point.p).reshape((-1,1))
            self.trajectory.v_release = get_release_velocity(self.hoop.p, self.trajectory.p_release, 1)
            
            self.trajectory.q_release = self.trajectory.tasktojoint(self.trajectory.p_release)
            
            # J = self.trajectory.Jv(self.trajectory.q_release)
            # self.trajectory.qdot_release = (np.transpose(J) @ np.linalg.inv(J @ np.transpose(J))) @ self.trajectory.v_release
            
            # Reset the trajectory
            self.t = 0
            self.trajectory.wait = False
            
     
        # Compute the desired joint positions and velocities for this time.
        (q, qdot, ptip, vtip) = self.trajectory.evaluate(self.t, self.dt)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = now.to_msg()      # Current time
        cmdmsg.name         = self.jointnames   # List of joint names
        cmdmsg.position     = q                 # List of joint positions
        cmdmsg.velocity     = qdot              # List of joint velocities
        self.pubs[0].publish(cmdmsg)
        
        ###################
        # Publish the basketball marker position
        if self.trajectory.ball_state == 'ground':
            # Ball is waiting to be picked up
            self.p.x, self.p.y, self.p.z = self.trajectory.p_ball_pile[0][0], self.trajectory.p_ball_pile[1][0], self.R
            self.v.x, self.v.y, self.v.z = 0.0, 0.0, 0.0
        elif self.trajectory.ball_state == 'holding':
            # Ball is on robotic arm
            self.p.x, self.p.y, self.p.z = ptip[0], ptip[1], ptip[2]
            self.v.x, self.v.y, self.v.z = vtip[0], vtip[1], vtip[2]
        elif self.trajectory.ball_state == 'free':
            # Ball is in free fall and not yet in hoop
            if not self.ball_in_hoop():
                self.v.z += self.dt * -9.81

                self.p.x += self.dt * self.v.x
                self.p.y += self.dt * self.v.y
                self.p.z += self.dt * self.v.z
            
        now = self.get_clock().now()
        self.marker.header.stamp = now.to_msg()
        self.pubs[1].publish(self.mark)
        ###################

#  Main Code
def main(args=None):
    # Initialize ROS and the node.
    rclpy.init(args=args)
    generator = Generator()

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted.
    rclpy.spin(generator)

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
