function F = ctlrRDHTforce(t,t_last,X,X_last,c,p,tau_des)
    % Find pressure on output piston
    %% WARNING: UNSTABLE!!!!
    tau_last = [0  0   0   0   p.kp*p.r            p.bp*p.r                -p.kp*p.r^2     -p.bp*p.r^2]*X_last;
    tau_out = [0  0   0   0   p.kp*p.r            p.bp*p.r                -p.kp*p.r^2     -p.bp*p.r^2]*X;
    
%     dt = t-t_last;
%     dFdt = (tau_out-tau_last)/dt;
    F = c.k*(tau_des-tau_out); %- c.d*dFdt;
%     F = 0;
end