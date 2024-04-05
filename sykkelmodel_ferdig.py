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
MOTORBREMS = 10 # [ns/m] ganget sammen med farten v utgjør dette en enkel tilmerming for motorbrems

# Vektinger for kostnadsfunksjonen
VEKTING_X = 1
VEKTING_Y = 1
VEKTING_V = 0

# "Straff" for endringer i inputs 
D_THROTTLE_PUNISHMENT = 1e-10
D_STEERING_PUNISHMENT = 1e-10

# Mpc parametere
PREDICTION_HORIZON = 50
TIME_STEP = 0.1
ROBUSTNESS = 1
STORE_SOLUTION = True

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
N_STEPS = 500

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
    rotasjon = (4*pi)/N_STEPS
    # Her generer vi input referanser, atm så er det en sirkel med radius *50* og *2* runder i vår simuleringstid
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
mpc.set_param(**setup_mpc)

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

# configuring tvp igjen? 
tvp_sim_template = simulator.get_tvp_template()

def tvp_sim_fun(t_now):
    return tvp_sim_template

simulator.set_tvp_fun(tvp_sim_fun)
simulator.setup()


# Initialverdier
x0 = np.array([X0, Y0, PSI0, V0, F0, DELTA0, DELTA_D0]).reshape(-1,1)
simulator.x0 = x0
mpc.x0 = x0

mpc.set_initial_guess()

# RUNNING THE SIMULATOR WITH CONTROL
simulator.reset_history()
simulator.x0 = x0
mpc.reset_history()
    
# Lag MPC steps, og simuler det
for i in range(N_STEPS):
    u0 = mpc.make_step(x0)
    x0 = simulator.make_step(u0)
    xref = xref_list[i]
    yref = yref_list[i]
    vref = vref_list[i]


def visualize_with_animation(path, xref_list, yref_list, vref_list):
    # Last resultater fra fil
    results = do_mpc.data.load_results(path)
    x_data = results['simulator']['_x']
    u_data = results['simulator']['_u']
    time = results['simulator']['_time']
    
    x = x_data[:,0]
    y = x_data[:,1]
    psi = x_data[:,2]
    v = x_data[:,3]
    f = x_data[:,4]
    delta = x_data[:,5]
    delta_der = x_data[:,6]

    throttle = u_data[:,0]
    steering_rate = u_data[:,1]

    # Lag figurer for XY, X, og Y
    fig, axs_xy = plt.subplots(4, 1, figsize=(10, 20))  

    # XY plot med referanse
    axs_xy[0].plot(x, y, label='Actual Path', color='blue')
    axs_xy[0].plot(xref_list, yref_list, label='Reference Path', linestyle='--', color='green')
    axs_xy[0].set_title('Position and Reference in XY')
    axs_xy[0].set_xlabel('x')
    axs_xy[0].set_ylabel('y')
    axs_xy[0].legend(loc='upper right') 
    axs_xy[0].grid(True)

    # Plot X og x_ref
    axs_xy[1].plot(time, x, label='x Actual', color='blue')
    axs_xy[1].plot(time, xref_list, label='x Reference', linestyle='--', color='green')
    axs_xy[1].set_title('X Position and Reference Over Time')
    axs_xy[1].set_xlabel('Time [s]')
    axs_xy[1].set_ylabel('X Position')
    axs_xy[1].legend(loc='upper right') 
    axs_xy[1].grid(True)

    # Plot y og y_ref
    axs_xy[2].plot(time, y, label='y Actual', color='blue')
    axs_xy[2].plot(time, yref_list, label='y Reference', linestyle='--', color='green')
    axs_xy[2].set_title('Y Position and Reference Over Time')
    axs_xy[2].set_xlabel('Time [s]')
    axs_xy[2].set_ylabel('Y Position')
    axs_xy[2].legend(loc='upper right')
    axs_xy[2].grid(True)

    # Plot fart og fart_ref
    axs_xy[3].plot(time, v, label='v Actual', color='blue')
    axs_xy[3].plot(time, vref_list, label='v Reference', linestyle='--', color='green')
    axs_xy[3].set_title('V Velocity and Reference Over Time')
    axs_xy[3].set_xlabel('Time [s]')
    axs_xy[3].set_ylabel('Velocity')
    axs_xy[3].legend(loc='upper right') 
    axs_xy[3].grid(True)

    plt.tight_layout(rect=[0, 0, 0.85, 1])  # Juster slik at det passer
    plt.show(block=False)

    
    
    # Lag figurer for f, styring, throttle, steering_rate
    fig, axs_inputs = plt.subplots(4, 1, figsize=(10, 20))

    # Plot f-kraft
    axs_inputs[0].plot(time, f, color='red')
    axs_inputs[0].set_title('Force over Time')
    axs_inputs[0].set_xlabel('Time [s]')
    axs_inputs[0].set_ylabel('Force [n]')
    axs_inputs[0].legend(loc='upper right')  
    axs_inputs[0].grid(True)

    # Plot styring
    axs_inputs[1].plot(time, delta, color='red')
    axs_inputs[1].set_title('Steeringangle delta')
    axs_inputs[1].set_xlabel('Time [s]')
    axs_inputs[1].set_ylabel('idk [rad]')
    axs_inputs[1].grid(True)

    # Plot throttle
    axs_inputs[2].plot(time, throttle, color='red')
    axs_inputs[2].set_title('throttle[INPUT]')
    axs_inputs[2].set_xlabel('Time [s]')
    axs_inputs[2].set_ylabel('throttle [N]')
    axs_inputs[2].grid(True)

    # Plot steering_rate
    axs_inputs[3].plot(time, steering_rate, color='red')
    axs_inputs[3].set_title('Steering_rate [INPUT]')
    axs_inputs[3].set_xlabel('Time [s]')
    axs_inputs[3].set_ylabel('Steering_rate [rad/s]')
    axs_inputs[3].grid(True)

    plt.tight_layout(rect=[0, 0, 0.85, 1]) # Juster slik at det passer
    plt.show(block=False)



    
    
    # Animasjon, tegner path i XY graf i "realtime"
    fig, ax = plt.subplots()
    ax.set_xlim(min(x)-10, max(x)+10)
    ax.set_ylim(min(y)-10, max(y)+10)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Position over Time')

    line, = ax.plot([], [], lw=2)
    ref_line, = ax.plot([], [], linestyle='--', color='green')

    def init():
        line.set_data([], [])
        ref_line.set_data([], [])
        return line, ref_line,
    
    skip = 3                                                     # Skip = 3 betyr at vi spiller bare hver 3 frame. høyere tall blir raskere spilling
    def animate(i):
        line.set_data(x[:i*skip], y[:i*skip])
        ref_line.set_data(xref_list[:i*skip], yref_list[:i*skip])
        return line, ref_line,

    fps = len(time) / (time[-1])
    ani = animation.FuncAnimation(fig, animate, frames=len(x)//skip, init_func=init, blit=True, interval=1000/fps)
    plt.show()




user_name = getpass.getuser()
file_path = os.path.join("C:", "Users", user_name, "results", "results.pkl")
# HVIS DETTE IKKE FUNGERER ER 
# file_path = r"C:\Users\vegar\results\results.pkl"
# Gamle hardkodet måten
if os.path.exists(file_path):
    os.remove(file_path)

do_mpc.data.save_results([simulator, mpc])
visualize_with_animation("results/results.pkl", xref_list, yref_list, vref_list)
