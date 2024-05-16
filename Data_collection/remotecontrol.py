#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64



import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)


class RemoteController(Node): 
   def __init__(self): 
      super().__init__("Remote_Control")
      self.speed_ = 0
      self.turn_ = 45
      self.brake_ = 0

      self.key = ' '

      self.throttle_pub = self.create_publisher(Int64,'/lw_portenta_throttle', 10)
      self.brake_pub = self.create_publisher(Int64, '/lw_portenta_brake', 10)
      self.steering_pub = self.create_publisher(Int64, '/lw_portenta_steering', 10)

      self.get_logger().info("Starting node")
      self.timer2 = self.create_timer(0.1, self.updateInputs)


   def updateInputs(self):
      self.key = self.getKey()
      msg_throttle = Int64()
      msg_brake = Int64()
      msg_steering = Int64()
      if self.key == 'w':
         if(self.speed_ < 15):
            self.speed_ += 1
            self.brake_ = 0
      if self.key == 'a':
         if(self.turn_ > 0):
            self.turn_ -= 1
      if self.key == 'd':
         if(self.turn_ < 90):
            self.turn_ += 1
      if self.key == 's':
         if self.speed_ > 0:
            self.speed_ -= 1
         if self.speed_ == 0:
            self.brake_ = 20
      if self.key == 'k':
         self.speed_ = 0
         self.brake_ = 20

      if self.key == 'q':
         rclpy.shutdown()



      msg_throttle.data = self.speed_
      msg_brake.data = self.brake_
      msg_steering.data = self.turn_


      self.throttle_pub.publish(msg_throttle)
      self.brake_pub.publish(msg_brake)
      self.steering_pub.publish(msg_steering)
      

      self.get_logger().info(self.key + '\t' + str(self.speed_) + '\t' + str(self.turn_) + '\t' + str(self.brake_))


      
            
   def getKey(self):
      tty.setraw(sys.stdin.fileno())
      select.select([sys.stdin], [], [], 0)
      self.key = sys.stdin.read(1)
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
      return self.key



def main(args=None):
   rclpy.init(args=args)
   
   node = RemoteController()

   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      print("Shutting down")
   finally:
      rclpy.shutdown()
   


if __name__ == '__main__':
   main()