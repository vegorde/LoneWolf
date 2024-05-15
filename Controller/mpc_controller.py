import casadi as ca
import numpy as np

class MPC_Controller:
    def __init__(self):
        # Punish refers to the R matrix. Reluctance to change control outputs.
        throttle_punish = 0.01
        steering_punish = 8
        
        # Bounds on control outputs.
        throttle_lower = 0
        throttle_upper = 100
        
        steering_lower = -np.pi/4
        steering_upper = np.pi/4

        self.dt = 0.1          # Sample time of the MPC.
        self.N = 30            # Prediciton horizon.
        self.L = 1.8           # Length of the wheelbase.
        self.M = 375           # Mass of Lone Wolf.

        self.tau = 1.5   # Time constant for filter simulating part of the engine dynamics,
        self.zeta = 0.88 # Zeta and omega are fitted to the steering response of the vehicle. See documentation for more info on all these parameters.
        self.omega = 8.56
        self.throttle_gain = 35
        self.ENGINEBRAKE = 225
        self.T_STEPS = self.dt

        #Internal variables for state estimator
        self.F_state = 0
        self.delta_state = 0
        self.delta_dot_state = 0

        self.throttle_state = 0
        self.steering_state = 0
        self.index = 0


        self.Q = ca.DM([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0]])

        self.R = ca.DM([[throttle_punish, 0],
                        [0, steering_punish]])




        #Defining states as casadi data structures
        self.x = ca.SX.sym('x', self.N+1)
        self.y = ca.SX.sym('y', self.N+1)
        self.psi = ca.SX.sym('psi', self.N+1)
        self.v = ca.SX.sym('v', self.N+1)
        self.F = ca.SX.sym('F', self.N+1)
        self.delta = ca.SX.sym('delta', self.N+1)
        self.delta_d = ca.SX.sym('delta_d', self.N+1)

        self.u1 = ca.SX.sym('u1', self.N)
        self.u2 = ca.SX.sym('u2', self.N)


        
        self.decision_vars = ca.vertcat(
            self.x,
            self.y,
            self.psi,
            self.v,
            self.F,
            self.delta,
            self.delta_d,
            self.u1,
            self.u2
        )


        self.lbx = ca.DM(np.full(self.decision_vars.size1(), -np.inf))  # Create a numpy array of -inf and convert to D       M
        self.ubx = ca.DM(np.full(self.decision_vars.size1(), np.inf))   # Create a numpy array of inf and convert to DM       



        
        #Finding the right index to start. 
        self.u1_start_index = 7*(self.N + 1)
        self.u2_start_index = self.u1_start_index + self.N 

        #Setting bounds
        self.lbx[self.u1_start_index:self.u1_start_index+self.N] = throttle_lower
        self.ubx[self.u1_start_index:self.u1_start_index+self.N] = throttle_upper

        self.lbx[self.u2_start_index:self.u2_start_index+self.N] = steering_lower
        self.ubx[self.u2_start_index:self.u2_start_index+self.N] = steering_upper 
  


    #State estimator functions
    def state_estimator(self, physical_states):
        x, y, psi, v = physical_states
        
        self.F_state += self.dt *( 1/self.tau * (-self.F_state + self.throttle_state * self.throttle_gain))
        self.delta_state += self.dt*(self.delta_dot_state)
        self.delta_dot_state += self.dt*(-2*self.zeta*self.omega * self.delta_dot_state - self.omega**2 * (self.delta_state - self.steering_state))

        return [x, y, psi, v, self.F_state, self.delta_state, self.delta_dot_state]




    def mpc_generate_step(self, physical_states, reference_list_x, reference_list_y):
        cost = 0
        g = []
        


        x_init, y_init, psi_init, v_init, F_init, delta_init, delta_dot_init = self.state_estimator(physical_states)

        # Initialize the initial condition vector
        initial_conditions = ca.DM.zeros(self.decision_vars.size1())

        # Set initial conditions for state variables at the first index
        initial_conditions[0 : self.N+1] = x_init  # Assuming all x values are initialized the same
        initial_conditions[self.N+1 : 2*(self.N+1)] = y_init
        initial_conditions[2*(self.N+1) : 3*(self.N+1)] = psi_init
        initial_conditions[3*(self.N+1) : 4*(self.N+1)] = v_init
        initial_conditions[4*(self.N+1) : 5*(self.N+1)] = F_init
        initial_conditions[5*(self.N+1) : 6*(self.N+1)] = delta_init
        initial_conditions[6*(self.N+1) : 7*(self.N+1)] = delta_dot_init
        initial_conditions[7*(self.N+1) : 7*(self.N+1) + self.N] = 50
        initial_conditions[7*(self.N+1) + self.N : 7*(self.N+1)+ 2*self.N] = 0
            

        #initial value for g
        g.append(self.x[0] - x_init)
        g.append(self.y[0] - y_init)
        g.append(self.psi[0] - psi_init)
        g.append(self.v[0] - v_init)
        g.append(self.F[0] - F_init)
        g.append(self.delta[0] - delta_init)
        g.append(self.delta_d[0] - delta_dot_init)



        for k in range(self.N):

            target_state = ca.DM([reference_list_x[k], reference_list_y[k], 0, 0])
            
            # Define 'state' as a CasADi vector directly
            state = ca.vertcat(self.x[k], self.y[k], self.psi[k], self.v[k])
            
            # Handling delta_control with CasADi expressions
            if k > 0:
                delta_control = ca.vertcat(self.u1[k], self.u2[k]) - ca.vertcat(self.u1[k-1], self.u2[k-1])
            else:
                delta_control = ca.vertcat(self.u1[k], self.u2[k])
            
            # Compute cost using CasADi's matrix operations
            state_diff = state - target_state
            cost += 0.5 * ca.mtimes([state_diff.T, self.Q, state_diff]) + 0.5 * ca.mtimes([delta_control.T, self.R, delta_control])


           
            # #discrete dynamic model
            x_next = self.x[k] + self.dt* self.v[k]*ca.cos(self.psi[k])
            y_next = self.y[k] + self.dt * self.v[k]*ca.sin(self.psi[k])
            psi_next = self.psi[k] + self.dt* self.v[k] / self.L * ca.tan(self.delta[k])
            v_next = self.v[k] + self.dt * (self.F[k]/self.M - self.ENGINEBRAKE/self.M*self.v[k])
            F_next = self.F[k] + self.dt/self.tau * (- self.F[k] + self.u1[k]*self.throttle_gain)
            delta_next = self.delta[k] + self.dt*self.delta_d[k]
            delta_d_next = self.delta_d[k] + self.dt  * ( -2 * self.zeta*self.omega*self.delta_d[k] - self.omega**2 * self.delta[k] + self.omega**2 * self.u2[k]) 
   

            if k < self.N-1: 
                g.append(self.x[k+1] - x_next)
                g.append(self.y[k+1] - y_next)
                g.append(self.psi[k+1] - psi_next)
                g.append(self.v[k+1] - v_next)
                g.append(self.F[k+1] - F_next)
                g.append(self.delta[k+1] - delta_next)
                g.append(self.delta_d[k+1] - delta_d_next)
        
        ipopt_options = {'print_level': 0, 'max_cpu_time': self.T_STEPS * 0.7}

        #Defining NLP and solver
        nlp = {'x': self.decision_vars, 'f': cost, 'g': ca.vertcat(*g)}
        solver = ca.nlpsol('solver', 'ipopt', nlp, {'ipopt': ipopt_options})



        sol = solver(x0=initial_conditions, lbx=self.lbx, ubx=self.ubx, lbg=0, ubg=0)

        optimized_u1_first = sol['x'][self.u1_start_index]
        optimized_u2_first = sol['x'][self.u2_start_index]

        self.throttle_state = optimized_u1_first
        self.steering_state = optimized_u2_first


        self.index += 1
        control_input_arr = [optimized_u1_first, optimized_u2_first, self.index]

        return control_input_arr
