function [A,b] = generate_2pt_corridor_constraint(path,traj_constant)

max_exponent = traj_constant.max_exponent;
cor_wid = traj_constant.cor_wid;
nc = traj_constant.nc;

A = [];
b = [];
%epsilon = 1e-06;
%nc = 100;

r_curr = path(1,1:3);
r_next = path(2,1:3);
t_curr = path(1,4);
t_next = path(2,4);
ti = (r_next - r_curr)/norm(r_next - r_curr);

for j = 1:nc
    
    t = t_curr+j*(t_next-t_curr)/(1+nc);
    time_vec = [];
    for i = 0:max_exponent
        time_vec = [time_vec t^i];
    end
    
    Axx = (1 - ti(1)^2)*time_vec;
    Axy = -ti(1)*ti(2)*time_vec;
    Axz = -ti(1)*ti(3)*time_vec;
    bx = -r_curr(1) + ti(1)*dot(r_curr,ti); 
    
    Ayx = -ti(1)*ti(2)*time_vec;
    Ayy = (1 - ti(2)^2)*time_vec;
    Ayz = -ti(2)*ti(3)*time_vec;
    by = -r_curr(2) + ti(2)*dot(r_curr,ti);
    
    Azx = -ti(1)*ti(3)*time_vec;
    Azy = -ti(2)*ti(3)*time_vec;
    Azz = (1 - ti(3)^2)*time_vec;
    bz = -r_curr(3) + ti(3)*dot(r_curr,ti);
    
    A_temp = [Axx Axy Axz;Ayx Ayy Ayz;Azx Azy Azz];
    A = [A; A_temp; -A_temp];
    
    b = [b;cor_wid-bx ;cor_wid-by ;cor_wid-bz;
           cor_wid+bx ;cor_wid+by ;cor_wid+bz];
    
end

end