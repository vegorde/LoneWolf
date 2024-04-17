def find_closest_point_on_path(x_current, y_current, x_ref, y_ref):
    # This function finds the closest point of the path and returns the index of this point.
    # Create an array of differences between reference points and the current point
    delta_x = np.array(x_ref) - x_current
    delta_y = np.array(y_ref) - y_current
    
    distances_to_path = np.sqrt(delta_x**2 + delta_y**2)
    nearest_index = np.argmin(distances_to_path)
    return nearest_index
