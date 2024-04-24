#!/usr/bin/env python3


import numpy as np
from scipy.optimize import minimize


class MPC_Controller:
   def __init__(self):

      self.L = 1.8  # Wheelbase
      self.D = 225  # Enginebreaks
      self.m = 375.0  # Mass
      self.tau = 1  # Time constant for filter 1
      self.zeta = 0.88 #Zeta and omega are fitted to the response of the vehicle. See documentation for more
      self.omega = 8.56
      self.throttle_gain = 35


      #Internal variables for state estimator
      self.F_state = 0
      self.delta_state = 0
      self.delta_dot_state = 0

      self.throttle_state = 0
      self.steering_state = 0



      #Tuning
      self.Q = np.array([[1,0,0,0,0,0,0],
                    [0,1,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],
                    [0,0,0,0,0,0,0],])

      self.R = np.array([[0,0],
                         [0,1]])



      self.dt = 0.1

      #Prediction horizon
      self.N = 15

      # Initial state
      self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, psi, v, F, delta, delta_dot]

      # Target state we want to reach
      self.target_state = np.array([100.0, 10.0, 0.0, 0, 0.0, 0.0, 0.0])


      self.control_sequence = np.zeros(2 * self.N)
      for i in range(len(self.control_sequence)):
         if (i % 2 == 0):
            self.control_sequence[i] = 100


      #Maximum steering and maximum throttle
      steering_bounds = (-np.pi/4, np.pi/4)
      throttle_bounds = (0,100)

      self.bounds = []    
      for _ in range(self.N):      

          self.bounds.append(throttle_bounds)  # Append acceleration bounds
          self.bounds.append(steering_bounds)      # Append steering bounds


   #State estimator functions
   def state_estimator(self, physical_states):
      x, y, psi, v = physical_states
      
      self.F_state += self.dt *( 1/self.tau * (-self.F_state + self.throttle_state * self.throttle_gain))
      self.delta_state += self.dt*(self.delta_dot_state)
      self.delta_dot_state += self.dt*(-2*self.zeta*self.omega * self.delta_dot_state - self.omega**2 * (self.delta_state - self.steering_state))

      return np.array([x, y, psi, v, self.F_state, self.delta_state, self.delta_dot_state])



   #Dynamic model
   def vehicle_dynamics(self, state, control):
         x, y, psi, v, F, delta, delta_dot = state
         u1, u2 = control
         dxdt = [v * np.cos(psi),
                 v * np.sin(psi),
                 v / self.L * np.tan(delta),
                 F/self.m - self.D/self.m * v,
                 -1/self.tau * F + 1/self.tau * u1*self.throttle_gain,
                 delta_dot,
                 -2*self.zeta*delta_dot*self.omega - delta*self.omega**2 + u2*self.omega**2]
         return np.array(dxdt)


   def mpc_cost_function(self, control_sequence, state, target_state, N, dt):

      cost = 0

      for i in range(N):
         control = control_sequence[i*2:(i+1)*2]  #Control sequence to be optimized 
         state = state + self.dt * self.vehicle_dynamics(state, control)
         if i > 0:
               previous_control = control_sequence[(i-1)*2:i*2]
               delta_control = control - previous_control
         else:
               delta_control = control
         cost += 1/2*(state - self.target_state)@self.Q@(state - self.target_state) + 1/2* delta_control @ self.R @ delta_control
      return cost


   def MPC_estimate_control_input(self, physical_states):
      state = self.state_estimator(physical_states)
      constraints = []


      res = minimize(self.mpc_cost_function, self.control_sequence, args=(state, self.target_state, self.N, self.dt),
                     method='SLSQP', bounds=self.bounds, constraints=constraints)

      #Get optimal control inputs
      optimal_control_sequence = res.x
      control_input = optimal_control_sequence[:2]

      #for testin purposes
      self.state = self.state + self.dt * self.vehicle_dynamics(self.state, control_input)

      # Update the initial guess for the next step (shifting the horizon)
      self.control_sequence = np.roll(self.control_sequence, -2)
      self.control_sequence[-2:] = control_input

      self.throttle_state, self.steering_state = control_input

      # print(f"Step {0}, State: {self.state}, Control: {control_input}")
      # print(res.x)
      # print(control_input)
      return control_input



# mpc = MPC_Controller()
# u = mpc.do_mpc([0,0,0,0])
# print(u)
