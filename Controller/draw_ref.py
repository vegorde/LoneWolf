import numpy as np
import matplotlib.pyplot as plt
"""
The drawing should begin at 0,0. The drawn shape will be completed in 50 seconds. try to keep the veloicty under 20m/s
by not drawing over 1000meters. 
Eventually increasing the num of points. This will need to be increased in Simulink [ros | variable size message | float64array] aswell
"""
class FreehandDrawer(object):
    def __init__(self, ax, num_points=500):
        self.ax = ax
        self.xy = []
        self.num_points = num_points
        self.line, = ax.plot([], [], 'k-')  # empty line
        self.ax.figure.canvas.mpl_connect('button_press_event', self.on_button_press)
        self.ax.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.ax.figure.canvas.mpl_connect('button_release_event', self.on_button_release)

    def on_button_press(self, event):
        if event.inaxes is not None:
            self.xy = [(event.xdata, event.ydata)]
            self.line.set_data(zip(*self.xy))
            self.ax.figure.canvas.draw()

    def on_motion(self, event):
        if event.button != 1 or event.inaxes is None:
            return
        self.xy.append((event.xdata, event.ydata))
        self.line.set_data(zip(*self.xy))
        self.ax.figure.canvas.draw()

    def on_button_release(self, event):
        if event.inaxes is not None:
            self.xy.append((event.xdata, event.ydata))
            self.line.set_data(zip(*self.xy))
            self.ax.figure.canvas.draw()
            # When the mouse is released, interpolate points
            self.interpolated_points = self.interpolate_points(self.xy, self.num_points)
            x_coords, y_coords = zip(*self.interpolated_points)  # Unzip the points
            print("xref_list =", list(x_coords))
            print("yref_list =", list(y_coords))
            print("return [xref_list, yref_list] \n\n\n\n\n")
            plt.figure()
            ax.plot(x_coords, y_coords, marker='o', markersize=6)
            plt.show()

    def interpolate_points(self, points, num_points):
        points = np.array(points)
        if len(points) < 2:
            return points.tolist()  # Avoid processing if not enough points
        # Linear length along the line
        distance = np.sqrt(np.diff(points[:,0])**2 + np.diff(points[:,1])**2)
        distance = np.insert(distance, 0, 0)
        cumulative_distance = np.cumsum(distance)
        # Interpolate values
        uniform_distances = np.linspace(0, cumulative_distance[-1], num_points)
        interpolated_x = np.interp(uniform_distances, cumulative_distance, points[:,0])
        interpolated_y = np.interp(uniform_distances, cumulative_distance, points[:,1])
        return list(zip(interpolated_x, interpolated_y))

fig, ax = plt.subplots(figsize=(10, 10))  # 10x10 inch window
ax.set_title('Draw here')
ax.set_xlim(-100, 100)  # Updated to start at -10
ax.set_ylim(-100, 100)
drawer = FreehandDrawer(ax)
plt.show()



