def find_closest_point_on_path(x_current, y_current, x_ref, y_ref):
    # Denne funksjonen må vi kalle i tvp for å erstatte timestep vi er på med kanskje timestep vi burde være på istedet
    
    # Også viktig at vi er relativt nerme der vi burde være i tid på pathen slik at vi ikke skipper en loop av en sirkel fek
    # hvis ref er kjør 2 runder vil denne kanskje sannynligvis hoppe på runde nr 2 med en gang
    # Mate inn i denne funksjonen den faktiske timestep vi er på ogaså vekte litt at vi burde være nerme dette timestepet
    # fek at distanse er distanse + delta timestep ? kanskjke tror dette kanskje fungerer vet ikke prøver å bigbraine
    
    # This function finds the closest point of the path and returns the index of this point.
    # Create an array of differences between reference points and the current point
    delta_x = np.array(x_ref) - x_current
    delta_y = np.array(y_ref) - y_current
    
    distances_to_path = np.sqrt(delta_x**2 + delta_y**2)
    nearest_index = np.argmin(distances_to_path)
    return nearest_index
