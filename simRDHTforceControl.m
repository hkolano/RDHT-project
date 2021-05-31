function [t_vec, X_vec] = simRDHTforceControl(X0,p, traj_fun, ctlr_fun) %  sol_set, mask] 
    %{
    Simulation script for RDHT simulation

    Inputs:
        X0: vector of initial conditions
        p: structure with system parameters
        traj_fun: external disturbance 
        ctlr_fun: controller function

    Modified: Hannah Kolano 2021
    %}

%     p.X_last = X0';
%     p.t_last = 0;

    % Running time
    t_start = 0;
    t_end = 3;
    dt = 0.1;

    t_vec = t_start:dt:t_end;

    % Simulation tolerances
    options = odeset(...
        'RelTol', 1e-5, ...
        'AbsTol', 1e-5);

    % Bind dynamics function
    dynamics_fun = @(t,X)dyn(t,X,p,traj_fun,ctlr_fun);
    
    % Run dynamics
    [t_vec, X_vec] = ode45(dynamics_fun, [t_start,t_end], X0, options);

end % simRDHT

function dX = dyn(t,X,p,traj_fun,ctlr_fun)
    % t == time
    % X == the state
    % p == parameters structure
%     Tau_ctrl = ctlr_fun(t,p.t_last,X,p.X_last); % Original force
%     controller
    Tau_ctrl = ctlr_fun(t,X);
%     Tau_ctrl = 0;
    [~, ~, ddy] = traj_fun(t);

    M1 = p.mw2*p.A1/p.a + p.mpd*p.a/p.A1;
    M2 = p.mw2*p.A2/p.a + p.mpd*p.a/p.A2;
    kpaOVA1  = p.kp*p.a/p.A1;
    bpaOVA1  = p.bp*p.a/p.A1;
    kpaOVA2  = p.kp*p.a/p.A2;
    bpaOVA2  = p.bp*p.a/p.A2;

%     if abs(X(3)) > p.strokelim 
%         k1 = 5000;
%         b1 = 10;
%     else
%         k1 = 0;
%         b1 = 0;
%     end
%     
%     if abs(X(5)) > p.strokelim
%         k2 = 5000;
%         b2 = 10;
%     else
%         k2 = 0;
%         b2 = 0;
%     end

    k1 = 0;
    k2 = 0;
    b1 = 0;
    b2 = 0;


    A = [0                 1                   0                           0                           0                           0                           0                   0; ...
        -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                           0                   0; ...
        0                   0                   0                           1                           0                           0                           0                   0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k1)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b1)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
        0                   0                   0                           0                           0                           1                           0                   0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
        0                   0                   0                           0                           0                           0                           0                   1; ...
        0                   0                   0                           0                           p.kp*p.r/p.Ip            p.bp*p.r/p.Ip                -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip];
%     A = [0                 1                   0                           0                           0                           0                           0                   0; ...
%             -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                           0                   0; ...
%             0                   0                   0                           1                           0                           0                           0                   0; ...
%             kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k1)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b1)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
%             0                   0                   0                           0                           0                           1                           0                   0; ...
%             0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a-k2)/M2  (-bpaOVA2-p.bf*p.A1/p.a-b2)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
%             0                   0                   0                           0                           0                           0                           0                   1; ...
%             0                   0                   0                           0                           0                           0                           0                   0];

    dX = A*X + [0; Tau_ctrl/p.Ip; 0; 0; 0; 0; 0; ddy];
%     p.X_last = X;
%     p.t_last = t;
end % dynamics
