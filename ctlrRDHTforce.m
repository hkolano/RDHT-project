function F = ctlrRDHTforce(t,X,c,p,tau_des,traj_fun)
    %% WARNING: UNSTABLE!!!!
    [~, ~, ddy] = traj_fun(t);
    tau_out = [0  0   0   0   p.kp*p.r    p.bp*p.r   -p.kp*p.r^2   -p.bp*p.r^2]*X - p.Ip*ddy;
    
    % Find desired linear force on belt
    F_des = tau_des/p.r;
    % Find desired compression of spring (delta_length)
    dl_des = F_des/p.kp;
    
    x_2 = X(5,:);
    dx_2 = X(6,:);
    theta_2 = X(7,:);
    dtheta_2 = X(8,:);
    
    F = -c.Kp*(dl_des-(theta_2*p.r-x_2))- ...
    c.Kd*(dtheta_2*p.r-dx_2);
    
end

%     Tau_des_Kevin = p.Ip*ddy + c.Kp*(dl_des-(theta_2*p.r-x_2))+ ...
%         c.Kd*(dtheta_2*p.r-dx_2);
%     tau_des_Kevin = tau_des + c.Kp*(dl_des-(theta_2*p.r-x_2))+ ...
%         c.Kd*(dtheta_2*p.r-dx_2);

%     F = c.k*(tau_des-tau_out) + tau_des; 
%     F = 0;