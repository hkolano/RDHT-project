function [t_vec, X_vec] = simRDHTballBounce(X0,p) %  sol_set, mask] 
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
    t_end = 2;
    dt = 0.005;

    t_vec = t_start:dt:t_end;
    X_vec = zeros(length(X0), length(t_vec));
    sol_set = {};
    
    free_event_fun = @(t,X)freeBallEvent(t,X,p);
    floor_event_fun = @(t,X)floorBallEvent(t,X,p);
    rod_event_fun = @(t,X)rodBallEvent(t,X,p);
    

    % Simulation tolerances
    optionsFree = odeset(...
        'RelTol', 1e-6, 'AbsTol', 1e-6, ...
        'Events', free_event_fun);
    optionsFloor = odeset(...
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', floor_event_fun);
    optionsRod = odeset(...
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', rod_event_fun);

    % Bind dynamics function
    free_dyn_fun = @(t,X)freedyn(t,X,p);
    ball_floor_dyn_fun = @(t,X)dyn_ballfloor(t,X,p);
    ball_rod_dyn_fun = @(t,X)dyn_ballrod(t,X,p);
    
    % States
    % 1: ball freefloating
    % 2: ball contacting ground
    % 3: ball contacting arm
    p.state = 1;


    while t_start < t_end
        % Simulate the dynamics over a time interval
        % Kept framework for hybrid dynamics
        if p.state == 1
            sol = ode45(free_dyn_fun, [t_start,t_end], X0, optionsFree);
            disp('Contact! at t = ')
            disp(sol.x(end))
            if sol.ie == 1 % if hits the ground
                disp('Hit the ground')
                coeffRest = 1;
                p.state = 2;
            else % if hits the rod
                disp('Hit the rod')
                coeffRest = 1;
                p.state = 3;
            end
        elseif p.state == 2
            sol = ode45(ball_floor_dyn_fun, [t_start,t_end], X0, optionsFloor);
            disp('Disconnected! at t = ')
            disp(sol.x(end))
%             disp('state after contact:')
%             disp(sol.y(10,end))
            coeffRest = 0.75;
            p.state = 1;
        else
            sol = ode45(ball_rod_dyn_fun, [t_start,t_end], X0, optionsRod);
            disp('Disconnected! at t = ')
            disp(sol.x(end))
%             disp('state after contact:')
%             disp(sol.y(10,end))
            coeffRest = 0.75;
            p.state = 1;
        end

        % Concatenate solution sets
        sol_set = [sol_set, {sol}];
        % Setup t_start for the next ode45 call so it is at the end of the
        % last call
        t_start = sol.x(end);
        % Set the initial conditions to the end of the last run
        if t_start == t_end
            X0 = [0 0 0 0 0 0 0 0 0 0];
        else
            X0 = sol.ye(:,end);
            X0(10) = sol.y(10,end)*coeffRest;
        end
    end
%
    % Loop to sample the solution structures and built X_vec
%     mask = zeros(1,length(t_vec));
    for idx = 1:length(sol_set)
        % This sets up a logical vector so we can perform logical indexing
        t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
%         disp('Is it empty?')
%         disp(any(t_sample_mask))
        % Evaluate the idx solution structure only at the applicable times
        if any(t_sample_mask)
            X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
            % Assign the result to the correct indicies of the return state array
            X_vec(:,t_sample_mask) = X_eval;
        end
    end

end % simRDHT

function dX = freedyn(t,X,p)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2,
    % y, y1)
    % p == parameters structure
    Tau_in = .25*cos(p.freq*t);
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
        0                   0                   0                           0                           p.kp*p.r/(p.Ip+p.Irod)      p.bp*p.r/(p.Ip+p.Irod)      -p.kp*p.r^2/(p.Ip+p.Irod)     -p.bp*p.r^2/(p.Ip+p.Irod)];
    dX_system = A*X(1:8) + [0; Tau_in/p.Ip; 0; 0; 0; 0; 0; 0];
    dX_ball = [0 1; 0 0]*X(9:10)+[0; -9.81];
    dX = [dX_system; dX_ball];
end % dynamics

%% Hybrid functions
function dX = dyn_ballfloor(t,X,p)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2,
    % y, y1)
    % p == parameters structure
    Tau_in = .25*cos(p.freq*t);
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
        0                   0                   0                           0                           p.kp*p.r/(p.Ip+p.Irod)      p.bp*p.r/(p.Ip+p.Irod)      -p.kp*p.r^2/(p.Ip+p.Irod)    -p.bp*p.r^2/(p.Ip+p.Irod)];
    dX_system = A*X(1:8) + [0; Tau_in/p.Ip; 0; 0; 0; 0; 0; 0];
    dX_ball = [0 1; -p.kball/p.mball 0]*X(9:10)+[0; -9.81];
    dX = [dX_system; dX_ball];
end % dynamics

%% Hybrid functions
function dX = dyn_ballrod(t,X,p)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2,
    % y, y1)
    % p == parameters structure
    Tau_in = .25*cos(p.freq*t);
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
    
    height_rod = p.h+0.75*p.l_rod*sin(X(7));
    dist = height_rod-X(9);


    A = [0                 1                   0                           0                           0                           0                           0                   0                0       0; ...
        -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                           0                   0                0       0; ...
        0                   0                   0                           1                           0                           0                           0                   0               0       0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b)/M1  p.kh*p.A2/(p.a*M1)      0                           0                   0               0       0; ...
        0                   0                   0                           0                           0                           1                           0                   0               0       0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2  0       0; ...
        0                   0                   0                           0                           0                           0                           0                   1               0       0; ...
        0                   0                   0                           0                           p.kp*p.r/(p.Ip+p.Irod)      p.bp*p.r/(p.Ip+p.Irod)      -p.kp*p.r^2/(p.Ip+p.Irod)    -p.bp*p.r^2/(p.Ip+p.Irod)        0       0; ...
        0                   0                   0                           0                           0                           0                           0                    0              0        1; ...
        0                   0                   0                           0                           0                           0                           0                0               0       0];
    dX = A*X + [0; Tau_in/p.Ip; 0; 0; 0; 0; 0; 0; 0; -9.81+p.kball*(dist)];

end % dynamics
%
function [eventVal, isterminal, direction] = freeBallEvent(t,X,p)
    % Inputs
    % t: time, X: the state, p: parameters structure
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    height_rod = p.h+0.75*p.l_rod*sin(X(7));
    dist = height_rod-X(9);
    eventVal = [X(9), dist];   % when spring at equilibrium distance...
    isterminal = [1, 1];     % stops the sim
    direction = [-1, -1];      % any direction
end
%
function [eventVal, isterminal, direction] = floorBallEvent(t,X,p)
% halting event function for ODE simulation. Events are distance to
    % ceiling and distance to paddle
    % Inputs
    % t: time, X: the state, p: parameters structure
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)

    eventVal =  X(9);   % when force from spring and damper = 0...
    isterminal = 1;     % stops the sim
    direction = 1;      % doesn't matter which direction
end

function [eventVal, isterminal, direction] = rodBallEvent(t,X,p)
% halting event function for ODE simulation. Events are distance to
    % ceiling and distance to paddle
    % Inputs
    % t: time, X: the state, p: parameters structure
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    height_rod = p.h+0.75*p.l_rod*sin(X(7));
    dist = height_rod-X(9);
    eventVal =  dist;   % when force from spring and damper = 0...
    isterminal = 1;     % stops the sim
    direction = 1;      % doesn't matter which direction
end


