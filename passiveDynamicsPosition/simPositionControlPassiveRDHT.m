function [t_vec, X_vec] = simPositionControlRDHTPassive(X0,p,c,freq, traj_fun, ctlr_fun)  %  sol_set, mask] 
    %{
    Simulation script for RDHT simulation

    Inputs:
        X0: vector of initial conditions
        p: structure with system parameters

    Author: Andrew Peekema
    Modified: Hannah Kolano 2021
    %}

    % Running time
    t_start = 0;
    t_end = 3;
    dt = 0.1;

    t_vec = t_start:dt:t_end;
    X_vec = zeros(length(X0), length(t_vec));
    sol_set = {};

    % Simulation tolerances
    options = odeset(...
        'RelTol', 1e-6, ...
        'AbsTol', 1e-6);

    % Bind dynamics function
    dynamics_fun = @(t,X)dyn(t,X,p,freq,traj_fun,ctlr_fun);


%     while t_start < t_end
        % Simulate the dynamics over a time interval
        % Kept framework for hybrid dynamics
%         if p.state == 1
      [t_vec, X_vec] = ode45(dynamics_fun, [t_start,t_end], X0, options);




end % simRDHT

function dX = dyn(t,X,p,freq,traj_fun,ctlr_fun)
    % t == time
    % X == the state
    % p == parameters structure
%     Tau_in = .5*cos(t);
%     Tau_in = 0;
    [~, ~, ddy] = traj_fun(t);

    Tau_ctrl = ctlr_fun(t,X,freq);
%     Tau_ctrl = 0;

    M1 = p.mw2*p.A1/p.a + p.mpd*p.a/p.A1;
    M2 = p.mw2*p.A2/p.a + p.mpd*p.a/p.A2;
    kpaOVA1  = p.kp*p.a/p.A1;
    bpaOVA1  = p.bp*p.a/p.A1;
    kpaOVA2  = p.kp*p.a/p.A2;
    bpaOVA2  = p.bp*p.a/p.A2;

    if abs(X(3)) > p.strokelim
        k = 5000;
        b = 10;
    else
        k = 0;
        b = 0;
    end


    A = [0                 1                   0                           0                           0                           0                           0                   0; ...
        -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                           0                   0; ...
        0                   0                   0                           1                           0                           0                           0                   0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
        0                   0                   0                           0                           0                           1                           0                   0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
        0                   0                   0                           0                           0                           0                           0                   1; ...
        0                   0                   0                           0                           p.kp*p.r/p.Ip            p.bp*p.r/p.Ip                -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip];
    B=[0; 1/p.Ip; 0; 0; 0; 0; 0; 0];
    E=[0; 0; 0; 0; 0; 0; 0; ddy];
    dX = A*X + B*Tau_ctrl+E;
end % dynamics

