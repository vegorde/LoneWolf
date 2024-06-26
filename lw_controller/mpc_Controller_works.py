#!/usr/bin/env python3

import numpy as np
import do_mpc
import time

class MPC_Controller:
    def __init__(self): 
        # Constants for our model used in the mpc and state estimator
        self.TAU = 1                             # Time constant of the first order lowpassfilter used to shape the rise curve of throttle vs speed [s]
        self.OMEGA = 8.56                        # Second order lowpassfilter used in calculating wheel angle from target wheel angle
        self.ZETA = 0.88                         # Still same filter, made to behave like a servo: Tune this with math if you're a nerd, else guess and check
        self.MASS = 375                          # Mass of LoneWolf [kg]
        self.L = 1.8                             # Lenght of wheelbase of LoneWolf [m]
        self.ENGINEBREAKS = 225                  # Resitance based on velocity. An opposing force [N/(m/s)]
        self.THROTTLEGAIN = 35                  # Tune throttlegain, enginebreaks, and tau to find an apropriate model with the same rise, steadystate, and fall based on experimental data


        # States for state-estimator
        self.F_state = 0
        self.delta_state = 0
        self.delta_dot_state = 0
        self.throttle_state = 0
        self.steering_target_state = 0
        self.physical_states = [0,0,0,0]
        self.x = 0
        self.y = 0

        # MPC Setup
        self.PREDICTION_HORIZON = 50         # PREDICTION_HORIZON * T_STEPS gives us the amount of seconds the mpc will look into the future when deciding control inputs: 
                                             # Larger horizon != better. Look in the terminal to see if the optimal solution is found :) or the cpu time limit is reached :(
        self.T_STEPS = 0.1                   # Timestep of the MPC
        self.ROBUST = 1                      # Robustness is a mesure of how well the mpc can handle modeling errors
        self.STORE_SOLUTION = False

        # Solver paramters
        self.MAX_ITERATIONS = 10000
        self.ABS_TOL = 1e-6                  # If we reach this tolerence before maxtime we will get a control output even earlier
        self.MAX_TIME = self.T_STEPS * 0.7   # [s]: This is max cpu time for the optimal sover. Important to garantee a control input within a timestep


        # CONSTANTS
        # Initial conditions of LoneWolf 
        X0, Y0, PSI0, V0, F0, D0, DD0 = [0, 0, 0, 0, 0, 0, 0]

        # Tuning parameters, how important it is to follow x and y part of reference
        X_WEIGHT = 1
        Y_WEIGHT = 1

        # R matrix, punishments for change in input
        THROTTLE_WEIGHT = 0.001      
        STEERING_TARGET_WEIGHT = 20#50 

        # Input bounds
        THROTTLE_LOW = 0      
        THROTTLE_HIGH = 100
        STEERING_LOW = -np.pi/180 * 45     # Limit for steering target
        STEERING_HIGH = np.pi/180 * 45 

        # State bounds
        V_LOW = 0                           # [m/s]  Actual top speed of real LoneWolf need to be here.
        V_HIGH = 16.67

        DELTA_LOW = -np.pi/180 * 45        # Limit for actual steering. Only difference between steering_target and delta is a second order lowpass filter made to behave like a servo
        DELTA_HIGH = np.pi/180 * 45        # [rad]



        # Defining the model used in the mpc
        model_type = 'continuous' # either 'discrete' or 'continuous'
        model = do_mpc.model.Model(model_type)

        # Setting state-variables, these are the variables we assume we can mesure
        x_pos = model.set_variable(var_type='_x', var_name='x_pos', shape=(1,1))            # X position in a global cordinate system
        y_pos = model.set_variable(var_type='_x', var_name='y_pos', shape=(1,1))            # Y position in a global cordinate system
        psi = model.set_variable(var_type='_x', var_name='psi', shape=(1,1))                # Jaw of lonewolf looking up from X axis
        v = model.set_variable(var_type='_x', var_name='v', shape=(1,1))                    # Velocity

        # Setting internal varables for the model.
        F = model.set_variable(var_type='_x', var_name='F', shape=(1,1))                    # Force acting on LoneWolf
        delta = model.set_variable(var_type='_x', var_name='delta', shape=(1,1))            # Wheel position relative to LoneWolf. Positive is left
        delta_dot = model.set_variable(var_type='_x', var_name='delta_dot', shape=(1,1))

        # Setting reference variables
        x_ref = model.set_variable(var_type='_tvp', var_name='x_ref', shape=(1,1))
        y_ref = model.set_variable(var_type='_tvp', var_name='y_ref', shape=(1,1))

        # Setting control input variablers
        Throttle = model.set_variable(var_type='_u', var_name='Throttle')
        Steering_target = model.set_variable(var_type='_u', var_name='Steering_target')
        

        # State-space model of LoneWolf
        x_dot = v*np.cos(psi)
        y_dot = v*np.sin(psi)
        psi_dot = v/self.L * np.tan(delta)
        v_dot = (F - self.ENGINEBREAKS * v)/self.MASS
        F_dot = 1/self.TAU * (-F + Throttle * self.THROTTLEGAIN)
        delta_d = delta_dot
        delta_ddot = -2*self.ZETA*self.OMEGA * delta_dot - self.OMEGA**2 * (delta - Steering_target)  

        model.set_rhs('x_pos', x_dot)
        model.set_rhs('y_pos', y_dot)
        model.set_rhs('psi', psi_dot)
        model.set_rhs('v', v_dot)
        model.set_rhs('F', F_dot)
        model.set_rhs('delta', delta_d)
        model.set_rhs('delta_dot', delta_ddot)

        model.setup()
        # The model is now set up :)
        

        # Generating a reference to follow. The velocity is baked into this reference as it is a parametriciced x(t), y(t).
        # When we reach the end of the reference it currenly loops, when path planning is integrated the reference will have no end.
        self.xref_list = []
        self.yref_list = []
        N_STEPS = 500   # N_STEPS has to be changed in simulink aswell. Read more in LoneWolfparams.m
        for i in range(N_STEPS):
            """
            # Sirkel ref
            antall_runder = 1
            rotasjon = (2*np.pi) * antall_runder /N_STEPS
            radius = 30
            self.xref_list.append(radius*np.cos(i*rotasjon))
            self.yref_list.append(radius*np.sin(i*rotasjon)) 
            """
            
            # Infinity ref
            # Drawing an infinity symbol with height = a, and with = 2a
            # Also calculating time and changing the timescale to be from [0,50] to [0,2pi] so the drawing works
            """     
            t = i * self.T_STEPS
            repetitions = 2
            t = t * (2*np.pi) * repetitions / (N_STEPS * self.T_STEPS)
            a = 50
            self.xref_list.append(a * np.sin(t))
            self.yref_list.append(a/2 * np.sin(2*t)) 
            """
            
            
            # Penis ref :)
            t = i * self.T_STEPS
            if t < 20:
                # Infinity symbol
                t = t * (2*np.pi) * 50/19 / (N_STEPS * self.T_STEPS)
                a = 30
                self.xref_list.append(a * np.sin(t))
                self.yref_list.append(a/2 * np.sin(2*t))
            elif t < 30:
                #Linje
                t = t - 20
                self.xref_list.append(10.0)
                self.yref_list.append(10 + 10*t)
            elif t < 40:
                # Sirkel
                t = (t - 30)
                r = 10
                w = (3*np.pi)/10 
                self.xref_list.append(r*np.cos(t*w))
                self.yref_list.append(110 + r*np.sin(t*w))
            elif t <= 49:
                # Linje
                t = t - 40
                self.xref_list.append(-10.0)
                self.yref_list.append(110 - 100/9*t)
            elif t <= 50:
                # Linje
                t = t - 49
                self.xref_list.append(-10.0 + 10*t)
                self.yref_list.append(10.0 - 10*t)
        
        self.xref_list_exstended = self.xref_list * 10
        self.yref_list_exstended = self.yref_list * 10

        # Setting up an mpc object
        self.mpc = do_mpc.controller.MPC(model)
        
        # These variables are explained above
        setup_mpc = {
            'n_horizon': self.PREDICTION_HORIZON,
            't_step': self.T_STEPS,
            'n_robust': self.ROBUST,
            'store_full_solution': self.STORE_SOLUTION,
        }
        
        # These variables are explained above
        solver_options = {
            'ipopt.max_iter': self.MAX_ITERATIONS,
            'ipopt.tol': self.ABS_TOL,        
            'ipopt.max_cpu_time': self.MAX_TIME 
        }

        combined_params = {**setup_mpc, 'nlpsol_opts': solver_options}
        self.mpc.set_param(**combined_params)



        # lterm is the objective function. What we are trying to minimize using the MPC
        lterm = (X_WEIGHT * (x_pos-x_ref)**2 + Y_WEIGHT * (y_pos-y_ref)**2)
        # lterm = (X_WEIGHT * np.abs((x_pos-x_ref)) + Y_WEIGHT * np.abs((y_pos-y_ref)))
        mterm = lterm*0   #This is the most scuffed way to say 0. I want it to be zero, as the mayer term focuses on final state, and there is no meaningfull final state

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        # rterm is the punishment for change in control inputs
        self.mpc.set_rterm(
            Throttle=THROTTLE_WEIGHT,
            Steering_target=STEERING_TARGET_WEIGHT
        )



        # Bounds on states:
        self.mpc.bounds['lower','_x', 'v'] = V_LOW
        self.mpc.bounds['upper','_x', 'v'] = V_HIGH
        
        self.mpc.bounds['lower','_x', 'delta'] = DELTA_LOW
        self.mpc.bounds['upper','_x', 'delta'] = DELTA_HIGH

        # Bounds on inputs:
        self.mpc.bounds['lower','_u', 'Throttle'] = THROTTLE_LOW
        self.mpc.bounds['upper','_u', 'Throttle'] = THROTTLE_HIGH
        
        self.mpc.bounds['lower','_u', 'Steering_target'] = STEERING_LOW
        self.mpc.bounds['upper','_u', 'Steering_target'] = STEERING_HIGH

        def find_closest_point_on_path(self, current_index):
            # This function finds the closest point of the path and returns the index of this point.
            # Create an array of differences between reference points and the current point
            vekting = 0.1 # større tall er mere vekting av tid, mindre av distanse :) 2 tilsvarer 20m/1s
                          # Lavere tall føler til bedre linjefølging. men ikke følging avpungt som beveger seg i tid

            distances = []
            for i in range(current_index + 1): # Sjekke nermeste pungtet på referanselinen som ikke er forran der vi er nå :)
                distances.append(np.sqrt((self.x - self.xref_list_exstended[i])**2 + (self.y - self.yref_list_exstended[i])**2) + vekting * np.abs(i - current_index)) # nermest i tid og distanse :)
            nearest_index = np.argmin(distances) 
            
            return nearest_index

        # Used in the MPC to see what the reference is in the future
        def tvp_fun(t_now):
            step = int(t_now // self.T_STEPS)                 # Calculate the current index based on the current time and the time step size.
            tvp_template = self.mpc.get_tvp_template()        # Initialize the tvp_template with the correct structure provided by do_mpc.


            closest_point_index = find_closest_point_on_path(self, step%(N_STEPS*10))
            for k in range(self.PREDICTION_HORIZON):          # Loop over the prediction horizon.
                ref_index = (closest_point_index + k) % len(self.xref_list)  # Index for the reference lists. Use modulo to cycle the references if the end is reached.

                # Set the TVP for the current step in the horizon.
                tvp_template['_tvp', k, 'x_ref'] = self.xref_list[ref_index]
                tvp_template['_tvp', k, 'y_ref'] = self.yref_list[ref_index]
            return tvp_template


        self.mpc.set_tvp_fun(tvp_fun)
        self.mpc.setup()


        x0_ = np.array([X0, Y0, PSI0, V0, F0, D0, DD0]).reshape(-1,1)   # Feed initial conditions
        self.mpc.reset_history()
        self.mpc.set_initial_guess()
        self.mpc.x0 = x0_
        self.mpc.set_initial_guess()

        self.previous_time = time.time()
        self.time_step = 0


    # This function is a helper-function that takes in the 4 measured states, and using this to estimate the internal states
    def state_estimator(self, physical_states):
        self.x, self.y, psi, v = physical_states
        dt = time.time() - self.previous_time
        self.F_state += dt *( 1/self.TAU * (-self.F_state + self.throttle_state * self.THROTTLEGAIN))
        self.delta_state += dt*(self.delta_dot_state)
        self.delta_dot_state += dt*(-2*self.ZETA*self.OMEGA * self.delta_dot_state - self.OMEGA**2 * (self.delta_state - self.steering_target_state))
        self.previous_time = time.time()

        return [self.x, self.y, psi, v, self.F_state, self.delta_state, self.delta_dot_state]



    def predict_step(self, physical_states):
        x0_temp = self.state_estimator(physical_states)
        x0_ = np.array(x0_temp)                                       # All states, mesured and estimated are fed into the mpc
        u = self.mpc.make_step(x0_)

        self.throttle_state = float(u[0])
        self.steering_target_state = float(u[1])
        self.time_step += 1                                           # Current time_step is sent to the node_script and published to simulink to know what reference is sent over time.
        return [u[0], (u[1]), self.time_step]

    
