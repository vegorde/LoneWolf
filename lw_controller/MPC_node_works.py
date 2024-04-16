#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from rclpy.duration import Duration
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Float64, Int64, Float64MultiArray

import time


from mpc_Controller_works import MPC_Controller
   


class mpc_ros_controller(Node):
   def __init__(self):
      super().__init__('mpc_controller')
      
      self.u_throttle = 0
      self.u_steering = 0

      #states
      self.x = 0
      self.y = 0
      self.psi = 0
      self.v = 0

      # #Internal filter variables (did this somewhere else, this is here for ref)
      # self.F = 0
      # self.delta = 0
      # self.delta_dot = 0

      #Timer
      self.time_start_ = time.time()
      self.time_now_ = 0 

      self.u1_publish_ = self.create_publisher(Int64, '/lw_portenta_throttle', 10)
      self.u2_publish_= self.create_publisher(Int64, '/lw_portenta_brake', 10)
      self.u3_publish_ = self.create_publisher(Int64, '/lw_portenta_steering', 10)
      self.time_publish_ = self.create_publisher(Int64, '/timestep', 10)
      
      x_ref_publish_ = self.create_publisher(Float64MultiArray, '/xref', 10)
      y_ref_publish_ = self.create_publisher(Float64MultiArray, '/yref', 10)



      self.xy_sub = self.create_subscription(NavSatFix, '/vectornav/gnss', self.update_states_xy, 10)
      self.psi_sub = self.create_subscription(PoseWithCovarianceStamped, '/vectornav/pose', self.update_states_psi, 10)
      self.v_sub = self.create_subscription(TwistWithCovarianceStamped, '/vectornav/velocity_body', self.update_states_v, 10)



      self.mpc = MPC_Controller()
      
      
      msg_x_ref = Float64MultiArray()
      msg_y_ref = Float64MultiArray()
      
      msg_x_ref.data = self.mpc.xref_list
      msg_y_ref.data = self.mpc.yref_list

      x_ref_publish_.publish(msg_x_ref)
      y_ref_publish_.publish(msg_y_ref)

      self.u1_spin = self.create_timer(0.1, self.generate_and_publish_inputs)

      

   def update_states_xy(self, msg): 
      self.x = msg.latitude
      self.y = msg.longitude
   

   def update_states_psi(self, msg): 
      self.psi = msg.pose.pose.position.z

   def update_states_v(self, msg): 
      self.v = msg.twist.twist.linear.x


   def generate_and_publish_inputs(self):
      physical_states = [self.x, self.y, self.psi, self.v]
      u_t = self.mpc.predict_step(physical_states)
      u_throttle = float(u_t[0])
      u_steering = float(u_t[1])
      time_step = int(u_t[2])

      msg_u1 = Int64()
      msg_u2 = Int64()
      msg_u3 = Int64()
      msg_timestep = Int64()

      msg_u1.data = int(u_throttle)
      msg_u2.data = 0
      msg_u3.data = int(u_steering*1000)
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
      #legg inn en fiks for at vi stopper hvis keyboard interrupt
      pass
      # save_data_to_cvs(node.time_stamp_, node.u1_list_, node.u2_list_,node.u3_list_, node.velocity_list_, node.angle_list_)

   finally: 
      rclpy.shutdown()


if __name__ == '__main__':
   main()