%{
ROB 542: Actuator Dyamics, Assignment 5 & 6

Rolling Diaphragm Hydrostatic Transmission simulation

Main code
Last modified by Hannah Kolano 5/24/21
%}

close all

%% Set up parameters

% cylinders: 25bar max
% Direct drive motor peak torque 19.6 Nm

% Geometry
p.r = 23.9/1000; % radius of the pulley, in mm (from Design and Experiment...)
p.A1 = (0.024/2)^2*pi; % Area of input piston, in m^2 (this is a little under a square inch)
p.A2 = (0.024/2)^2*pi; % Area of ouput piston, in m^2
p.a = (.006/2)^2*pi; % Area of the tube, in m^2
p.strokelim = 56.8/2/1000; % Stroke length limit (either way from 0)

% Masses
mp = 0.05;  % Pulley mass
p.Ip = 0.5*mp*p.r^2;   % pulley inertia
p.mpd = 0.05;  % Piston and diaphragm mass, kg
p.mw = 0.1;   % Total mass of the water
p.mw2 = p.mw/2;   % Mass of half the water

% Stiffnesses
p.kp = 2014000; % Stiffness of the belt N/m 
p.kh = 1573;   % Stiffness of the hose N/m of y1

% Damping
p.bp = .5;     % Damping of the belt
p.bf = 2.137;     % Viscous friction N/(m/s) of y1

% External disturbance
p.dist_amp = .5; % Amplitude of disturbance: ~30 degrees
p.dist_freq = 0.25; % Frequency of disturbance, Hz

traj_fun = @(t) disturbanceTrajectory(p.dist_amp, p.dist_freq, t);

% Set up controller
c.Kp = 5000;
c.Kd =30;
c.k = 1.5;
tau_des = 0;

% set up controller
ctlr_fun = @(t,X) ctlrRDHTforce(t,X,c,p,tau_des, traj_fun);

%% Simulate the system
X0 = [.5 0 .5*p.r 0 .5*p.r 0 .5 0];

[t_vec, X_vec] = simRDHTforceControl(X0,p, traj_fun, ctlr_fun);


%% retrieve control force
force = ctlr_fun(t_vec',X_vec');
figure
plot(t_vec, force)
xlabel('Time (s)')
ylabel('Controller Force (Nm)')
title('Controller Output')

%% Retrieve output forces
[~, ~, ddy] = traj_fun(t_vec);
tau_out = [0  0   0   0   p.kp*p.r    p.bp*p.r   -p.kp*p.r^2   -p.bp*p.r^2]*X_vec'- p.Ip*ddy';
% output_tau = calculate_output_torque(t_vec, X_vec, traj_fun)
figure
plot(t_vec, tau_out,  'b-', 'LineWidth', 2)
ylabel('Output Force (Nm)')
xlabel('Time (s)')
title('Torque on Output')


%% Plotting
figure
% plot(t_vec, X_vec(1,:))
plot(t_vec, X_vec(:,1));
hold on
plot(t_vec, X_vec(:,7))
% plot(t_vec, 5*sin(t_vec))
legend('Theta1', 'Theta2')
xlabel('Time (s)')
ylabel('Radians')
title('Angular Displacement')

% figure
% plot(t_vec, X_vec(:,3))
% hold on
% plot(t_vec, X_vec(:,5))
% legend('x1', 'x2')
% xlabel('Time (s)')
% ylabel('Displacement (m)')
% title('Piston Displacement')

% figure
% 
% plot(t_vec, X_vec(:,7)-X_vec(:,1))
% xlabel('Time (s)')
% ylabel('Position error (rad)')
% title('Input-Output shaft')

%Animate
% exportVideo = false;
% playbackRate = 1;
% RDHTAnimation(p,t_vec,X_vec,exportVideo,playbackRate);

