#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int64, Float64MultiArray
import numpy as np
# from mpc_jorgen import MPC_Controller           # Coustom class
# from mpc_jorgen_linearized import MPC_Controller
from mpc_jorgen_ipopt import MPC_Controller
   

class mpc_ros_controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        self.u_throttle = 0
        self.u_steering = 0

        # Mesured states of LoneWolf
        self.x = 0
        self.y = 0
        self.psi = 0
        self.v = 0

        self.u1_publish_ = self.create_publisher(Int64, '/lw_portenta_throttle', 10)
        self.u2_publish_= self.create_publisher(Int64, '/lw_portenta_brake', 10)
        self.u3_publish_ = self.create_publisher(Int64, '/lw_portenta_steering', 10)
        self.time_publish_ = self.create_publisher(Int64, '/timestep', 10)
        
        self.xy_sub = self.create_subscription(NavSatFix, '/vectornav/gnss', self.update_states_xy, 10)
        self.psi_sub = self.create_subscription(PoseWithCovarianceStamped, '/vectornav/pose', self.update_states_psi, 10)
        self.v_sub = self.create_subscription(TwistWithCovarianceStamped, '/vectornav/velocity_body', self.update_states_v, 10)
        
        # Creating an mpc object using out coustom mpc class
        self.mpc = MPC_Controller()
        sampletime = self.mpc.T_STEPS
        self.time_step = 0
        self.timestep = 0
        
        self.xref_list = []
        self.yref_list = []
        N_STEPS = 500   # N_STEPS has to be changed in simulink aswell. Read more in LoneWolfparams.m
        for i in range(N_STEPS):
            # Infinity ref
            # Drawing an infinity symbol with height = a, and with = 2a
            # Also calculating time and changing the timescale to be from [0,50] to [0,2pi] so the drawing works         
            t = i * sampletime
            repetitions = 1
            t = t * (2*np.pi) * repetitions / (N_STEPS * sampletime)
            a = 50
            self.xref_list.append(a * np.sin(t))
            self.yref_list.append(a/2 * np.sin(2*t))
            # self.xref_list.append(t*20/3.6)
            # self.yref_list.append(0.0)

        # Since the x and y ref are [currently] complete pre generated lists that do not change we only need to publish this once, so all the publish logic is in __init__
        x_ref_publish_ = self.create_publisher(Float64MultiArray, '/xref', 10)
        y_ref_publish_ = self.create_publisher(Float64MultiArray, '/yref', 10)
        msg_x_ref = Float64MultiArray()
        msg_y_ref = Float64MultiArray()
        msg_x_ref.data = self.xref_list
        msg_y_ref.data = self.yref_list
        
        import time
        for _ in range(10):
            x_ref_publish_.publish(msg_x_ref)
            y_ref_publish_.publish(msg_y_ref)
            time.sleep(0.1)

        # Creating a spin node that uses our states to generate controller outputs and publishes these to ROS2
        self.publisher_spin_ = self.create_timer(sampletime, self.generate_and_publish_inputs)


    

    def update_states_xy(self, msg): 
        self.x = msg.latitude
        self.y = msg.longitude
   

    def update_states_psi(self, msg): 
        self.psi = msg.pose.pose.position.z

    def update_states_v(self, msg): 
        self.v = msg.twist.twist.linear.x


    def generate_and_publish_inputs(self):
        physical_states = [self.x, self.y, self.psi, self.v]    # Mesured states from ROS
        
        start_index = find_closest_point_on_path(self, self.time_step%5000)
        xref_n = []
        yref_n = []
        for i in range(start_index, start_index + self.mpc.N):
            i = i%500
            xref_n.append(self.xref_list[i])
            yref_n.append(self.yref_list[i])
        for _ in range(1):
            print(start_index)    
            print(xref_n[0])
            print(yref_n[0])
        u_t = self.mpc.mpc_generate_step(physical_states, xref_n, yref_n)            # Calculated internal states + mesured states used to calculate a control input
        u_throttle = float(u_t[0])
        u_steering = float(u_t[1])
        self.time_step = int(u_t[2])


        msg_u1 = Int64()                                        # Throttle
        msg_u2 = Int64()                                        # Brake
        msg_u3 = Int64()                                        # Steering
        msg_timestep = Int64()

        msg_u1.data = int(u_throttle)                           
        msg_u2.data = 0                                         # Currently not using brake as a control input
        msg_u3.data = int(u_steering*1000)                      # Since steering is in rad currently and we send ints. we scale this up before sending, then scale it down upon reciving
        msg_timestep.data = self.time_step

        self.get_logger().info("Sending inputs")
        self.u1_publish_.publish(msg_u1)
        self.u2_publish_.publish(msg_u2)
        self.u3_publish_.publish(msg_u3)
        self.time_publish_.publish(msg_timestep)

def find_closest_point_on_path(self, current_index):
        # This function finds the closest point of the path and returns the index of this point.
        # Create an array of differences between reference points and the current point
        vekting = 0.1 # større tall er mere vekting av tid, mindre av distanse :) 2 tilsvarer 20m/1s
                        # Lavere tall føler til bedre linjefølging. men ikke følging avpungt som beveger seg i tid

        distances = []
        for i in range(current_index + 1): # Sjekke nermeste pungtet på referanselinen som ikke er forran der vi er nå :)
            distances.append(np.sqrt((self.x - (self.xref_list*10)[i])**2 + (self.y - (self.yref_list*10)[i])**2) + vekting * np.abs(i - current_index)) # nermest i tid og distanse :)
        nearest_index = np.argmin(distances) 
        return nearest_index




def main(args=None):
    rclpy.init(args=args)
    node = mpc_ros_controller()
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Properly handle ROS node shutdown on keyboard interrupt [ctrl + c]
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down node.')
        node.destroy_node()  # This will clean up the resources used by the node.
    finally:
        rclpy.shutdown()     # This will stop the ROS client library


if __name__ == '__main__':
   main()
