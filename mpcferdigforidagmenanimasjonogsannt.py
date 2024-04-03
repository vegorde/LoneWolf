import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import do_mpc
import os
import matplotlib.animation as animation

# Customizing Matplotlib:
mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True


# MODELING OF SYSTEM
model_type = 'continuous'
model = do_mpc.model.Model(model_type)

x_pos = model.set_variable(var_type='_x', var_name='x_pos', shape=(1,1))
y_pos = model.set_variable(var_type='_x', var_name='y_pos', shape=(1,1))
theta = model.set_variable(var_type='_x', var_name='theta', shape=(1,1))
velocity = model.set_variable(var_type='_x', var_name='velocity', shape=(1,1))

acceleration = model.set_variable(var_type='_u', var_name='acceleration')
delta = model.set_variable(var_type='_u', var_name='delta')
5
# time-varying parameters
x_ref = model.set_variable(var_type='_tvp', var_name='x_ref')
y_ref = model.set_variable(var_type='_tvp', var_name='y_ref')
velocity_ref = model.set_variable(var_type='_tvp', var_name='velocity_ref')

# defining and setting r.h.s
x_pos_der = np.cos(theta)*velocity
y_pos_der = np.sin(theta)*velocity
theta_der = delta
velocity_der = acceleration

model.set_rhs('x_pos', x_pos_der)
model.set_rhs('y_pos', y_pos_der)
model.set_rhs('theta', theta_der)
model.set_rhs('velocity', velocity_der)

model.setup()


# CONFIGURING MPC CONTROLLER
mpc = do_mpc.controller.MPC(model)

# parameter
setup_mpc = {
    'n_horizon': 20,
    't_step': 0.1,
    'n_robust': 1,
    'store_full_solution': True,
}
mpc.set_param(**setup_mpc)

# objective function
mterm = (x_pos-x_ref)**2 + (y_pos-y_ref)**2 + 0*(velocity-velocity_ref)**2
lterm = mterm

mpc.set_objective(mterm=mterm, lterm=lterm)

mpc.set_rterm(  #penalty for input changes
    acceleration= 5,  # kankje 1e-2 er realistisk idk
    delta = 5
)

# lower bounds on inputs:
mpc.bounds['lower','_u', 'acceleration'] = -5
mpc.bounds['lower','_u', 'delta'] = -np.pi/4

# upper bounds on inputs:
mpc.bounds['upper','_u', 'acceleration'] = 5
mpc.bounds['upper','_u', 'delta'] = np.pi/4

# time varying parameters
tvp_template = mpc.get_tvp_template()

#initial referanser i think

xref_list = []
yref_list = []
vref_list = []
n_steps = 1500
for i in range(n_steps):
    if i < 1000:
        """
        a = 250
        xref_list.append(a*np.cos(i*0.01)/(1+np.sin(i*0.01)**2))
        yref_list.append(a*np.cos(i*0.01)*np.sin(i*0.01)/(1+np.sin(i*0.01)**2))
        vref_list.append(5)
        """
        xref_list.append(50*np.cos(i*0.01))
        yref_list.append(50*np.sin(i*0.01))
        vref_list.append(0)
    else:
        xref_list.append(i - 1000)
        yref_list.append(-25)
        vref_list.append(0)


def tvp_fun(t_now):
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

mpc.set_tvp_fun(tvp_fun)

mpc.setup()


# CONFIGURING THE SIMULATOR
simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step = 0.1)

# configuring tvp
tvp_sim_template = simulator.get_tvp_template()

def tvp_sim_fun(t_now):
    return tvp_sim_template

simulator.set_tvp_fun(tvp_sim_fun)

simulator.setup()


# CREATING THE CONTROL LOOP INITIAL CONDITIONS
x0 = 0
y0 = 20
thetha0 = 3/2*np.pi
v0 = 10

x0 = np.array([x0,y0,thetha0, v0]).reshape(-1,1)

simulator.x0 = x0
mpc.x0 = x0

mpc.set_initial_guess()


# RUNNING THE SIMULATOR WITH CONTROL
simulator.reset_history()
simulator.x0 = x0
mpc.reset_history()
    

for i in range(n_steps):
    u0 = mpc.make_step(x0)
    x0 = simulator.make_step(u0)
    xref = xref_list[i]
    yref = yref_list[i]
    vref = vref_list[i]


def visualize_with_animation(pathiboy, xref_list, yref_list, vref_list):
    # Load results
    results = do_mpc.data.load_results(pathiboy)
    x_data = results['simulator']['_x']
    u_data = results['simulator']['_u']
    time = results['simulator']['_time']
    x_pos = x_data[:,0]
    y_pos = x_data[:,1]
    theta = x_data[:,2]
    velocity = x_data[:,3]

    # Assuming 'acceleration' is the first control input (u_data[:,0])
    # and 'delta' is the second control input (u_data[:,1]), which represent velocity_dot and theta_dot
    acceleration = u_data[:,0]
    delta = u_data[:,1]

    # Create figure and axes for XY plot
    fig, axs_xy = plt.subplots(3, 1, figsize=(10, 20))  

    # Plot XY position with references
    axs_xy[0].plot(x_pos, y_pos, label='Actual Path', color='blue')
    axs_xy[0].plot(xref_list, yref_list, label='Reference Path', linestyle='--', color='green')
    axs_xy[0].set_title('Position and Reference in XY')
    axs_xy[0].set_xlabel('x')
    axs_xy[0].set_ylabel('y')
    axs_xy[0].legend(loc='upper right')  # Move legend outside of plot
    axs_xy[0].grid(True)

    # Plot x and x_ref over time
    axs_xy[1].plot(time, x_pos, label='x Actual', color='blue')
    axs_xy[1].plot(time, xref_list, label='x Reference', linestyle='--', color='green')
    axs_xy[1].set_title('X Position and Reference Over Time')
    axs_xy[1].set_xlabel('Time [s]')
    axs_xy[1].set_ylabel('X Position')
    axs_xy[1].legend(loc='upper right')  # Move legend outside of plot
    axs_xy[1].grid(True)

    # Plot y and y_ref over time
    axs_xy[2].plot(time, y_pos, label='y Actual', color='blue')
    axs_xy[2].plot(time, yref_list, label='y Reference', linestyle='--', color='green')
    axs_xy[2].set_title('Y Position and Reference Over Time')
    axs_xy[2].set_xlabel('Time [s]')
    axs_xy[2].set_ylabel('Y Position')
    axs_xy[2].legend(loc='upper right')  # Move legend outside of plot
    axs_xy[2].grid(True)

    plt.tight_layout(rect=[0, 0, 0.85, 1])  # Adjust the rect to prevent overlap between the subplots and the legend
    plt.show(block=False)

    # Create figure and axes for inputs plot
    fig, axs_inputs = plt.subplots(4, 1, figsize=(10, 20))

    # Plot velocity over time
    axs_inputs[0].plot(time, velocity, color='red')
    axs_inputs[0].plot(time, vref_list, label='Velocity Reference', linestyle='--', color='green') 
    axs_inputs[0].set_title('Velocity and Reference over Time')
    axs_inputs[0].set_xlabel('Time [s]')
    axs_inputs[0].set_ylabel('Velocity [m/s]')
    axs_inputs[0].legend(loc='upper right')  # Move legend outside of plot
    axs_inputs[0].grid(True)

    # Plot acceleration over time
    axs_inputs[1].plot(time, acceleration, color='red')
    axs_inputs[1].set_title('Acceleration over Time')
    axs_inputs[1].set_xlabel('Time [s]')
    axs_inputs[1].set_ylabel('Acceleration [m/s²]')
    axs_inputs[1].grid(True)

    # Plot delta over time
    axs_inputs[2].plot(time, delta, color='red')
    axs_inputs[2].set_title('Delta over Time')
    axs_inputs[2].set_xlabel('Time [s]')
    axs_inputs[2].set_ylabel('Delta [rad]')
    axs_inputs[2].grid(True)

    # Plot theta over time
    axs_inputs[3].plot(time, theta, color='red')
    axs_inputs[3].set_title('Theta over Time')
    axs_inputs[3].set_xlabel('Time [s]')
    axs_inputs[3].set_ylabel('Theta [rad]')
    axs_inputs[3].grid(True)

    plt.tight_layout(rect=[0, 0, 0.85, 1])  # Adjust the rect to prevent overlap between the subplots and the legend
    plt.show(block=False)

    # Animation
    fig, ax = plt.subplots()
    ax.set_xlim(min(x_pos), max(x_pos))
    ax.set_ylim(min(y_pos), max(y_pos))
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Position over Time')

    line, = ax.plot([], [], lw=2)
    ref_line, = ax.plot([], [], linestyle='--', color='green')


    def init():
        line.set_data([], [])
        ref_line.set_data([], [])
        return line, ref_line,
    skip = 1
    def animate(i):
        skip = 1
        line.set_data(x_pos[:i*skip], y_pos[:i*skip])
        ref_line.set_data(xref_list[:i*skip], yref_list[:i*skip])
        return line, ref_line,

    # Determine frames per second (fps) based on speedup_factor
    fps = len(time) / (time[-1])
    
    ani = animation.FuncAnimation(fig, animate, frames=len(x_pos)//skip, init_func=init, blit=True, interval=1000/fps)
    plt.show()





file_path = r"C:\Users\vegar\results\results.pkl" #dette må være din egen filnavn ofc
if os.path.exists(file_path):
    os.remove(file_path)

do_mpc.data.save_results([simulator, mpc])
visualize_with_animation("results/results.pkl", xref_list, yref_list, vref_list)