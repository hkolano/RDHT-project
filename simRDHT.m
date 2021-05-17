function [t_vec, X_vec, sol_set, mask] = simRDHT(X0,p)
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
    t_end = 50;
    dt = 0.5;

    t_vec = t_start:dt:t_end;
    X_vec = zeros(length(X0), length(t_vec));
    sol_set = {};

    % Simulation tolerances
    options = odeset(...
        'RelTol', 1e-9, ...
        'AbsTol', 1e-9);

    % Bind dynamics function
    dynamics_fun = @(t,X)dyn(t,X,p);

    while t_start < t_end
        % Simulate the dynamics over a time interval
        % Kept framework for hybrid dynamics
%         if p.state == 1
            sol = ode45(dynamics_fun, [t_start,t_end], X0, options);


        sol_set = [sol_set, {sol}];
        % Setup t_start for the next ode45 call so it is at the end of the
        % last call
        t_start = sol.x(end);
        % Set the initial conditions to the end of the last run
        if t_start == t_end
            X0 = [0 0 0 0 0 0 0 0];
        else
            X0 = sol.ye(:,end);
        end
    end

    % Loop to sample the solution structures and built X_vec
    mask = zeros(1,length(t_vec));
    for idx = 1:length(sol_set)
        % This sets up a logical vector so we can perform logical indexing
        t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
        % Evaluate the idx solution structure only at the applicable times
        X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
        % Assign the result to the correct indicies of the return state array
        X_vec(:,t_sample_mask) = X_eval;

        if rem(idx,2) == 0
            mask(t_sample_mask) = t_sample_mask(t_sample_mask);
        end
    end
    mask = logical(mask);

end % simRDHT

function dX = dyn(t,X,p)
    % t == time
    % X == the state
    % p == parameters structure
   fq=2;
   a=5;
    Tau_in = -a*fq*fq*cos(t*fq)

    M1 = p.mw2*p.A1/p.a + p.mpd*p.a/p.A1;
    M2 = p.mw2*p.A2/p.a + p.mpd*p.a/p.A2;
    kpaOVA1  = p.kp*p.a/p.A1;
    bpaOVA1  = p.bp*p.a/p.A1;
    kpaOVA2  = p.kp*p.a/p.A2;
    bpaOVA2  = p.bp*p.a/p.A2;

    A = [0                 1                   0                           0                           0                           0                           0                   0; ...
        -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                           0                   0; ...
        0                   0                   0                           1                           0                           0                           0                   0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a)/M1  (-bpaOVA1-p.bf*p.A1/p.a)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
        0                   0                   0                           0                           0                           1                           0                   0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
        0                   0                   0                           0                           0                           0                           0                   1; ...
        0                   0                   0                           0                           p.kp*p.r/p.Ip            p.bp*p.r/p.Ip                -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip];
%      A = [0                 1                   0                           0                           0                           0                           0                   0; ...
%         -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                            0                   0; ...
%         0                   0                   0                           1                           0                           0                           0                   0; ...
%         p.kp*p.r/p.mpd      p.bp*p.r/p.mpd      -p.kp/p.mpd                 -p.bp/p.mpd                 0                           0                           0                   0; ...
%         0                   0                   0                           0                           0                           1                           0                   0; ...
%         0                   0                   0                           0                           0                           0                           0                   0; ...
%         0                   0                   0                           0                           0                           0                           0                   1; ...
%         0                   0                   0                           0                           0                           0                           0                   0];

dX = A*X + [0; Tau_in; 0; 0; 0; 0; 0; 0];
end % dynamics
