import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import do_mpc
import os
import getpass
import matplotlib.animation as animation
from numpy import cos, sin, tan, pi


TAU = 1 # tidskonstant for 1 ordens lavpass for force
ZETA = 1 # demping for delta
OMEGA = 1 # udempet svingefrekvens for andre ordens filter delta
LEN = 1 # lengde fra hjul til hjul
M = 375 # masse på LW
MOTORBREMS = 100 # [ns/m] ganget sammen med farten v utgjør dette en enkel tilmerming for motorbrems

# Vektinger for kostnadsfunksjonen
VEKTING_X = 1
VEKTING_Y = 1
VEKTING_V = 0

# "Straff" for endringer i inputs 
D_THROTTLE_PUNISHMENT = 1e-10
D_STEERING_PUNISHMENT = 1e-10

# Mpc parametere
PREDICTION_HORIZON = 40
TIME_STEP = 0.1
ROBUSTNESS = 1
STORE_SOLUTION = False

# Solver parametere
MAX_ITERATIONS = 10000
ABS_TOL = 1e-6
MAX_TIME = 0.07 # Dette holder oss innenfor 0.1 sekunder

# Grenser på inputs
THROTTLE_LOW = 0       # Throttle [?N?]
THROTTLE_HIGH = 5000
STEERING_RATE_LOW = -pi/4  # [?rad/s?]
STEERING_RATE_HIGH = pi/4

# Grenser på states
V_LOW = 0                 # Hastighet [m/s]
V_HIGH = 100

DELTA_LOW = -pi/4         # Hjulvinkel [rad]
DELTA_HIGH = pi/4

# For generering av referanse og simulering og sånt. simulerings tid blir N_STEPS * TIME_STEP (0.1)
N_STEPS = 250

# Initialverdier på states
X0 = 0
Y0 = 0
PSI0 = 0
V0 = 0
F0 = 0
DELTA0 = 0
DELTA_D0 = 0

# Matplotlib parameter
mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True






# Modelering av systemet
model_type = 'continuous'
model = do_mpc.model.Model(model_type)

# States
x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
psi = model.set_variable(var_type='_x', var_name='psi', shape=(1,1))
v = model.set_variable(var_type='_x', var_name='v', shape=(1,1))
f = model.set_variable(var_type='_x', var_name='f', shape=(1,1))
delta = model.set_variable(var_type='_x', var_name='delta', shape=(1,1))
delta_der = model.set_variable(var_type='_x', var_name='delta_der', shape=(1,1))

# Inputs
throttle = model.set_variable(var_type='_u', var_name='throttle')
steering_rate = model.set_variable(var_type='_u', var_name='steering_rate')

# Referanser
x_ref = model.set_variable(var_type='_tvp', var_name='x_ref')
y_ref = model.set_variable(var_type='_tvp', var_name='y_ref')
velocity_ref = model.set_variable(var_type='_tvp', var_name='velocity_ref')

# ODE Systemet i state space
x_d = v * cos(psi)
y_d = v * sin(psi)
psi_d = v/LEN * tan(delta)
v_d = f/M
f_d = 1/TAU * (-f + throttle) - MOTORBREMS * v
delta_d = delta_der
delta_der_d = -2 * ZETA * OMEGA * delta_der - OMEGA**2 * delta + steering_rate

model.set_rhs('x', x_d)
model.set_rhs('y', y_d)
model.set_rhs('psi', psi_d)
model.set_rhs('v', v_d)
model.set_rhs('f', f_d)
model.set_rhs('delta', delta_d)
model.set_rhs('delta_der', delta_der_d)

model.setup()






# Lag referanse
xref_list = []
yref_list = []
vref_list = []
for i in range(N_STEPS):
    rotasjon = (2*pi)/N_STEPS
    # Her generer vi input referanser, atm så er det en sirkel med radius *50* og *1* runder i vår simuleringstid
    r = 50
    xref_list.append(r*cos(i*rotasjon))
    yref_list.append(r*sin(i*rotasjon))
    vref_list.append(0)







# Sett opp MPC
mpc = do_mpc.controller.MPC(model)
setup_mpc = {
    'n_horizon': PREDICTION_HORIZON,
    't_step': TIME_STEP,
    'n_robust': ROBUSTNESS,
    'store_full_solution': STORE_SOLUTION,
}
solver_options = {
    'ipopt.max_iter': MAX_ITERATIONS,   # Maximum number of iterations for IPOPT
    'ipopt.tol': ABS_TOL,        # Solver tolerance for IPOPT
    'ipopt.max_cpu_time': MAX_TIME # Maximum CPU time limit in seconds for IPOPT
}

combined_params = {**setup_mpc, 'nlpsol_opts': solver_options}
mpc.set_param(**combined_params)

# Kostnadsfunksjon
mterm = VEKTING_X * (x-x_ref)**2 + VEKTING_Y * (y-y_ref)**2 + VEKTING_V * (v-velocity_ref)**2
lterm = mterm
mpc.set_objective(mterm=mterm, lterm=lterm)

# Staff for input endringer
mpc.set_rterm( 
    throttle= D_THROTTLE_PUNISHMENT,
    steering_rate = D_STEERING_PUNISHMENT
)

# Grenser på inputs
mpc.bounds['lower','_u', 'throttle'] = THROTTLE_LOW
mpc.bounds['upper','_u', 'throttle'] = THROTTLE_HIGH
mpc.bounds['upper','_u', 'steering_rate'] = STEERING_RATE_HIGH
mpc.bounds['lower','_u', 'steering_rate'] = STEERING_RATE_LOW

# Grenser på states
mpc.bounds['lower','_x', 'v'] = V_LOW
mpc.bounds['upper','_x', 'v'] = V_HIGH
mpc.bounds['lower','_x', 'delta'] = DELTA_LOW
mpc.bounds['upper','_x', 'delta'] = DELTA_HIGH


def tvp_fun(t_now):
    #Jeg vet ikke hva dette er. Dette er chatgpt sitt verk enn så lenge. Må lese litt på dokumentasjonen til DO_MPC
    
    # Calculate the current index based on the current time and the time step size.
    step = int(t_now // setup_mpc['t_step'])

    # Initialize the tvp_template with the correct structure provided by do_mpc.
    tvp_template = mpc.get_tvp_template()

    # Loop over the prediction horizon.
    for k in range(setup_mpc['n_horizon']):
        # Index for the reference lists. Use modulo to cycle the references if the end is reached.
        ref_index = (step + k) % len(xref_list)

        # Set the TVP for the current step in the horizon.
        tvp_template['_tvp', k, 'x_ref'] = xref_list[ref_index]
        tvp_template['_tvp', k, 'y_ref'] = yref_list[ref_index]
        tvp_template['_tvp', k, 'velocity_ref'] = vref_list[ref_index]

    return tvp_template

tvp_template = mpc.get_tvp_template()
mpc.set_tvp_fun(tvp_fun)
mpc.setup()
# Vi har nå mpc :)

# Simulator
simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step = TIME_STEP)


def tvp_sim_fun(t_now):
    return tvp_sim_template



# Initialverdier
x0 = np.array([X0, Y0, PSI0, V0, F0, DELTA0, DELTA_D0]).reshape(-1,1)
mpc.x0 = x0

mpc.set_initial_guess()

mpc.reset_history()



def simulate_step(X0, Y0, PSI0, V0, F0, DELTA0, DELTA_D0, throttle, steering_rate, dt):
    for _ in range(100):
        x_d = V0 * np.cos(PSI0)
        y_d = V0 * np.sin(PSI0)
        psi_d = V0 / LEN * np.tan(DELTA0)
        v_d = F0 / M
        f_d = 1 / TAU * (-F0 + throttle) - MOTORBREMS * V0
        delta_d = DELTA_D0
        delta_der_d = -2 * ZETA * OMEGA * DELTA_D0 - OMEGA**2 * DELTA0 + steering_rate
        
        X0 += x_d * dt
        Y0 += y_d * dt
        PSI0 += psi_d * dt
        V0 += v_d * dt
        F0 += f_d * dt
        DELTA0 += delta_d * dt
        DELTA_D0 += delta_der_d * dt
    
    return np.array([X0, Y0, PSI0, V0, F0, DELTA0, DELTA_D0])


X0_, Y0_, PSI0_, V0_, F0_, DELTA0_, DELTA_D0_ = 0, 0, 0, 0, 0, 0, 0

states = []
throttle_values = []
import time
times = []
# Lag MPC steps, og simuler det
for i in range(N_STEPS):
    T0 = time.time()
    u0 = mpc.make_step(x0)
    T1 = time.time()
    throttle_ = u0[0]
    steering_rate_ = u0[1]
    
    x0 = simulate_step(X0_, Y0_, PSI0_, V0_, F0_, DELTA0_, DELTA_D0_, throttle_, steering_rate_, TIME_STEP/100)
    states.append(x0)
    X0_, Y0_, PSI0_, V0_, F0_, DELTA0_, DELTA_D0_ = x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6]
    xref = xref_list[i]
    yref = yref_list[i]
    vref = vref_list[i]
    times.append(T1-T0)
    throttle_values.append(throttle_)





data = states  # Your data goes here
# Extract X1 and Y1 values
# Extract X1, Y1, and Time for plotting
X1 = [item[0] for item in states]
Y1 = [item[1] for item in states]
velocity_values = [item[3] for item in states]

Time = [i * TIME_STEP for i in range(N_STEPS)]

# Plotting XY path, throttle, and velocity
fig, axs = plt.subplots(3, 1, figsize=(10, 15), constrained_layout=True)

# Plot X1 vs Y1
axs[0].plot(X1, Y1, '--', label='Actual Path')  # Actual path
axs[0].plot(xref_list, yref_list, '--', label='Reference Path')  # Reference path
axs[0].set_title('Actual vs Reference Path')
axs[0].set_xlabel('X1')
axs[0].set_ylabel('Y1')
axs[0].legend()
axs[0].grid(True)

# Plot Throttle over Time
axs[1].plot(Time, throttle_values, label='Throttle')
axs[1].set_title('Throttle Over Time')
axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('Throttle')
axs[1].legend()
axs[1].grid(True)

# Plot Velocity over Time
axs[2].plot(Time, velocity_values, label='Velocity')
axs[2].set_title('Velocity Over Time')
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('Velocity [m/s]')
axs[2].legend()
axs[2].grid(True)

print(times)
plt.show()
