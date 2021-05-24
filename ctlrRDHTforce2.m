function F = ctlrRDHTforce2(t,X,c,p,tau_des)
    % Find desired linear force on belt
    F_des = tau_des/p.r;
    % Find desired compression of spring (delta_length)
    dl_des = F_des/p.kp;
    
    x_2 = X(5,:);
    dx_2 = X(6,:);
    theta_2 = X(7,:);
    dtheta_2 = X(8,:);
    
%     curr_dl = theta_2*p.r-x_2
    
%     F = c.Kp*(dl_des-(theta_2*p.r-x_2))+ ...
%         c.Kd*(dtheta_2*p.r-dx_2);
    F = .1;
%     F = 0;
end