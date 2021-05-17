function [t_vec, X_vec] = simRDHT(X0,p) %  sol_set, mask] 
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
    t_end = 6;
    dt = 0.1;

    t_vec = t_start:dt:t_end;
    X_vec = zeros(length(X0), length(t_vec));
    sol_set = {};

    % Simulation tolerances
    options = odeset(...
        'RelTol', 1e-6, ...
        'AbsTol', 1e-6);
    
    % Bind dynamics function
    dynamics_fun = @(t,X)dyn(t,X,p);
   

%     while t_start < t_end
        % Simulate the dynamics over a time interval
        % Kept framework for hybrid dynamics
%         if p.state == 1
      [t_vec, X_vec] = ode45(dynamics_fun, [t_start,t_end], X0, options);
%             disp('Contact! at t = ')
%             disp(sol.x(end))
%             p.state = 0;
%         else 
%             sol = ode45(conn_dyn, [t_start,t_end], X0, contactoptions);
%             disp('Disconnected! at t = ')
%             disp(sol.x(end))
%             p.state = 1;
%         end

        % Concatenate solution sets
%         sol_set = [sol_set, {sol}];
%         % Setup t_start for the next ode45 call so it is at the end of the 
%         % last call 
%         t_start = sol.x(end);
%         % Set the initial conditions to the end of the last run
%         if t_start == t_end
%             X0 = [0 0 0 0 0 0 0 0];
%         else
%             X0 = sol.ye(:,end);
%         end
%     end
%     
%     % Loop to sample the solution structures and built X_vec
%     mask = zeros(1,length(t_vec));
%     for idx = 1:length(sol_set)
%         % This sets up a logical vector so we can perform logical indexing
%         t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
%         % Evaluate the idx solution structure only at the applicable times
%         X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
%         % Assign the result to the correct indicies of the return state array
%         X_vec(:,t_sample_mask) = X_eval;
%         
%         if rem(idx,2) == 0
%             mask(t_sample_mask) = t_sample_mask(t_sample_mask);
%         end
%     end
%     mask = logical(mask);

end % simRDHT

function dX = dyn(t,X,p)
    % t == time
    % X == the state
    % p == parameters structure
    Tau_in = 1*cos(t);
%     Tau_in = 0;
    
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
%      A = [0                 1                   0                           0                           0                           0                           0                   0; ...
%         -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip     p.bp*p.r/p.Ip               0                           0                            0                   0; ...
%         0                   0                   0                  1                           0                           0                           0                   0; ...
%         p.kp*p.r/p.mpd      p.bp*p.r/p.mpd      -p.kp/p.mpd        -p.bp/p.mpd                 0                           0                           0                   0; ...
%         0                   0                   0                           0                           0                           1                           0                   0; ...
%         0                   0                   0                           0                           0                           0                           0                   0; ...
%         0                   0                   0                           0                           0                           0                           0                   1; ...
%         0                   0                   0                           0                           0                           0                           0                   0];
    dX = A*X + [0; Tau_in/p.Ip; 0; 0; 0; 0; 0; 0];
end % dynamics

function k = pistonLimitSpring(x, p)
    if abs(x) > p.strokelim
        k = 10;
    else
        k = 0;
    end
end

%% Hybrid functions
% function dX = connecteddynamics(t,X,p,ctlr_fun)
%     % t == time
%     % X == the state
%     % p == parameters structure
%     F_ctrl = ctlr_fun(t,X);
%     
%     y_act  = X(1); % Position of actuator
%     y_load = X(2); % Position of load
%     dy_act = X(3); % Velocity of actuator
%     dy_load = X(4); % Velocity of load
% 
%     % Return the state derivative
%     dX = zeros(4,1);
%     dX(1) = dy_act;     % Velocity of actuator
%     dX(2) = dy_load;    % Velocity of load
%     dX(3) = (F_ctrl + p.k*(y_load-y_act) + p.c*(dy_load-dy_act))/p.actm;          % Acceleration of actuator
%     dX(4) = ( - p.k*(y_load-y_act) - p.c*(dy_load-dy_act))/p.loadm; % Acceleration of load
% end % dynamics
% 
% function [eventVal, isterminal, direction] = contactSpringEvent(t,X,p)
% % halting event function for ODE simulation. Events are distance to
%     % ceiling and distance to paddle
%     % Inputs
%     % t: time, X: the state, p: parameters structure
%     % Outputs
%     % eventVal: Vector of event functions that halt at zero crossings
%     % isterminal: if the simulation should halt (yes for both)
%     % direction: which direction of crossing should the sim halt (positive)
%     equil = X(2)-X(1); % distance between load and actuator, 0 when spring is at equilibrium
%     
%     eventVal = equil;   % when spring at equilibrium distance...
%     isterminal = 1;     % stops the sim
%     direction = 0;      % any direction
% end
% 
% function [eventVal, isterminal, direction] = liftoffSpringEvent(t,X,p)
% % halting event function for ODE simulation. Events are distance to
%     % ceiling and distance to paddle
%     % Inputs
%     % t: time, X: the state, p: parameters structure
%     % Outputs
%     % eventVal: Vector of event functions that halt at zero crossings
%     % isterminal: if the simulation should halt (yes for both)
%     % direction: which direction of crossing should the sim halt (positive)
%     y_act  = X(1); % Position of actuator
%     y_load = X(2); % Position of load
%     dy_act = X(3); % Velocity of actuator
%     dy_load = X(4); % Velocity of load
%    
%     eventVal =  - p.k*(y_load-y_act) - p.c*(dy_load-dy_act);   % when force from spring and damper = 0...
%     isterminal = 1;     % stops the sim
%     direction = -1;      % doesn't matter which direction
% end
