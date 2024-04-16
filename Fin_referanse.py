for i in range(N_STEPS):
    t = i * T_STEPS
    if t < 20:
        # Infinity symbol
        t = t * (2*np.pi) * 50/20 / (N_STEPS * self.T_STEPS)
        a = 30
        self.xref_list.append(a * np.sin(t))
        self.yref_list.append(a/2 * np.sin(2*t))
    elif t < 30:
        #Linje
        t = t - 20
        self.xref_list.append(10)
        self.yref_list.append(10 + 10*t)
    elif t < 40:
        # Sirkel
        t = (t - 30)
        r = 10
        w = (3*np.pi)/10 
        self.xref_list.append(r*np.cos(t*w))
        self.yref_list.append(110 + r*np.sin(t*w))
    elif t < 50:
        # Linje
        t = t - 40
        self.xref_list.append(-10)
        self.yref_list.append(110 - 10*t)
