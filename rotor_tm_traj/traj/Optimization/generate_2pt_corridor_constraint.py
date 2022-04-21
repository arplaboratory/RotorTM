import numpy as np
# tested
def generate_2pt_corridor_constraint(path, traj_constant):
    # returns A and b

    max_exponent = traj_constant.max_exponent
    cor_wid = traj_constant.cor_wid
    nc = traj_constant.nc

    A = np.empty((0, (max_exponent+1)*3))
    b = np.empty((0,1))

    r_curr = path[0,0:3]
    r_next = path[1,0:3]
    t_curr = path[0,3]
    t_next = path[1,3]
    ti = (r_next - r_curr)/np.linalg.norm(r_next - r_curr)

    for j in range(1, nc+1):
        
        t = t_curr+j*(t_next-t_curr)/(1+nc)
        time_vec = np.array([[]])
        for i in range(0, max_exponent+1):
            time_vec = np.hstack((time_vec, np.array([[t**i]])))
        
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
        A_temp = np.vstack((np.hstack((Axx, Axy, Axz)), np.hstack((Ayx, Ayy, Ayz)), np.hstack((Azx, Azy, Azz))))
        A = np.vstack((A, A_temp, -A_temp))
        test = (cor_wid-bx)
        b = np.vstack(  (
                        b,
                        np.array([[cor_wid-bx]]),
                        np.array([[cor_wid-by]]),
                        np.array([[cor_wid-bz]]),
                        np.array([[cor_wid+bx]]),
                        np.array([[cor_wid+by]]),
                        np.array([[cor_wid+bz]])
                        )
                     )
    
    return A, b