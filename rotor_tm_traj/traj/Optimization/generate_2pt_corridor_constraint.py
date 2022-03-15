import numpy as np
# untested
def generate_2pt_corridor_constraint(path, traj_constant):
    # returns A and b

    max_exponent = traj_constant["max_exponent"]
    cor_wid = traj_constant["cor_wid"]
    nc = traj_constant["nc"]

    A = np.array([[]])
    b = np.array([[]])
    # epsilon = 1e-06;
    # nc = 100;

    r_curr = path[0,0:2]
    r_next = path[1,0:2]
    t_curr = path[0,3]
    t_next = path[1,3]
    ti = (r_next - r_curr)/np.linalg.norm(r_next - r_curr)

    for j in range(1, nc+1):
        
        t = t_curr+j*(t_next-t_curr)/(1+nc)
        time_vec = np.array([[]])
        for i in range(0, max_exponent+1):
            time_vec = np.append(time_vec, t**i, axis=1)
        
        Axx = (1 - ti[0]**2)*time_vec
        Axy = -ti[0]*ti[1]*time_vec
        Axz = -ti[0]*ti[2]*time_vec
        bx = -r_curr[0] + ti[0]*np.dot(r_curr,ti); 
        
        Ayx = -ti[0]*ti[1]*time_vec
        Ayy = (1 - ti[1]**2)*time_vec
        Ayz = -ti[1]*ti[2]*time_vec
        by = -r_curr[1] + ti[1]*np.dot(r_curr,ti)
        
        Azx = -ti[0]*ti[2]*time_vec
        Azy = -ti[1]*ti[2]*time_vec
        Azz = (1 - ti[2]**2)*time_vec
        bz = -r_curr[2] + ti[2]*np.dot(r_curr,ti)
        
        A_temp = np.stack(np.array([Axx, Axy, Axz]), np.array([Ayx, Ayy, Ayz]), np.array([Azx, Azy, Azz]), axis=1)
        A = np.stack(A, A_temp, -A_temp, axis=1)
        
        b = np.stack(b, cor_wid-bx, cor_wid-by, cor_wid-bz, cor_wid+bx, cor_wid+by, cor_wid+bz)

        result = {}
        result["A"] = A
        result["b"] = b
        return A, b