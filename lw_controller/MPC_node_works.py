#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int64, Float64MultiArray

from mpc_Controller_works import MPC_Controller            # Coustom class
   

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
      
      # Since the x and y ref are [currently] complete pre generated lists that do not change we only need to publish this once, so all the publish logic is in __init__
      x_ref_publish_ = self.create_publisher(Float64MultiArray, '/xref', 10)
      y_ref_publish_ = self.create_publisher(Float64MultiArray, '/yref', 10)
      msg_x_ref = Float64MultiArray()
      msg_y_ref = Float64MultiArray()
      msg_x_ref.data = self.mpc.xref_list
      msg_y_ref.data = self.mpc.yref_list
      x_ref_publish_.publish(msg_x_ref)
      y_ref_publish_.publish(msg_y_ref)

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
      u_t = self.mpc.predict_step(physical_states)            # Calculated internal states + mesured states used to calculate a control input
      
      u_throttle = float(u_t[0])
      u_steering = float(u_t[1])
      time_step = int(u_t[2])

      msg_u1 = Int64()                                        # Throttle
      msg_u2 = Int64()                                        # Brake
      msg_u3 = Int64()                                        # Steering
      msg_timestep = Int64()

      msg_u1.data = int(u_throttle)                           
      msg_u2.data = 0                                         # Currently not using brake as a control input
      msg_u3.data = int(u_steering*1000)                      # Since steering is in rad currently and we send ints. we scale this up before sending, then scale it down upon reciving
      msg_timestep.data = time_step

      self.get_logger().info("Sending inputs")
      self.u1_publish_.publish(msg_u1)
      self.u2_publish_.publish(msg_u2)
      self.u3_publish_.publish(msg_u3)
      self.time_publish_.publish(msg_timestep)






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
