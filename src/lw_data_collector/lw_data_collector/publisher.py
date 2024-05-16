#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int64
from geometry_msgs.msg import TwistWithCovarianceStamped
import time
import math as m
import pandas as pd
import argparse

class Publisher(Node):

   def __init__(self, input_list1, input_list2, input_list3):
      super().__init__("data_publisher")
      self.u1_ = input_list1
      self.u2_ = input_list2
      self.u3_ = input_list3
      self.counter_ = 0
      self.time_start_ = time.time()
      self.time_now_ = 0 #I might not need this at the moment, this was used for the cosine function
      self.u1_publish_ = self.create_publisher(Int64, '/lw_portenta_throttle', 10)
      self.u2_publish_= self.create_publisher(Int64, '/lw_portenta_brake', 10)
      self.u3_publish_ = self.create_publisher(Int64, '/lw_portenta_steering', 10)
      self.mock_publish = self.create_publisher(TwistWithCovarianceStamped, '/vectornav/velocity_body', 10)
      # self.mock_states = self.create_publisher(Twist, '/vectornav/velocity_body', 10)
      self.get_logger().info("Publishing inputs")
      self.u1_spin = self.create_timer(0.1, self.publish_inputs)
      self.mock = self.create_timer(0.2, self.publish_mock_states)
   
   def publish_inputs(self):
      msg_u1 = Int64()
      msg_u2 = Int64()
      msg_u3 = Int64()
      msg_u1.data = self.u1_[self.counter_]
      msg_u2.data = self.u2_[self.counter_]
      msg_u3.data = self.u3_[self.counter_]
      self.counter_ += 1
      self.get_logger().info("Sending inputs")
      self.u1_publish_.publish(msg_u1)
      self.u2_publish_.publish(msg_u2)
      self.u3_publish_.publish(msg_u3)
      
   #Denne er kun for test idag. Resten fungerer fint.   
   def publish_mock_states(self): 
      self.time_now_ = time.time() - self.time_start_ 
      msg = TwistWithCovarianceStamped()
      msg.twist.twist.linear.x = m.cos(self.time_now_)
      msg.twist.twist.angular.z = m.sin(2*self.time_now_)
      self.get_logger().info("twist states published")
      self.mock_publish.publish(msg)




def main(script):
   nr = str(script)
   print(f"~/input_data_ntnu/input{nr}.csv")
   input_data = pd.read_csv(f"~/input_data_ntnu/input{nr}.csv", delimiter=';')
   input_data['u1'] = input_data['u1'].astype(int)
   input_data['u2'] = input_data['u2'].astype(int)
   input_data['u3'] = input_data['u3'].astype(int)

   list_of_u1 = input_data['u1'].tolist()
   list_of_u2 = input_data['u2'].tolist()
   list_of_u3 = input_data['u3'].tolist()


   rclpy.init(args=None)

   node = Publisher(list_of_u1,list_of_u2,list_of_u3)
   rclpy.spin(node)

   rclpy.shutdown()

if __name__ == '__main__':
   parser = argparse.ArgumentParser(description='scriptvalg')
   parser.add_argument('script', help='hvilken fil csvfil')
   args = parser.parse_args()
   main(args.script)