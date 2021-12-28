function [mu, nullspace_coeff] = optimal_cable_distribution(P_bar, rho_vec, mu_min_norm, params,coeff_init)

fun = @(x)quadobj(x,P_bar'*P_bar,P_bar'*mu_min_norm,0);
nonlconstr = @(x)cable_distribution_constr(x,rho_vec,params.cable_length,P_bar,mu_min_norm,params.robot_radius);
if nargin == 5
coeff_opt = fmincon(fun,coeff_init,[],[],[],[],[],[],nonlconstr);
elseif nargin == 6
coeff_opt = fmincon(fun,params.coeff0,[],[],[],[],[],[],nonlconstr);
end

%coeff_opt = quadprog(P_bar_world'*P_bar_world,cable_tensions'*P_bar_world);
mu = mu_min_norm + P_bar * coeff_opt;
nullspace_coeff =  coeff_opt;

end
