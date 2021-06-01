%{
ROB 542: Actuator Dyamics, Assignment 7

Rolling Diaphragm Hydrostatic Transmission simulation

Main code
Last modified by Hannah Kolano 5/30/21
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

p.h = 0.5; % Distance between pulley center and ground

% Masses
mp = 0.05;  % Pulley mass
p.Ip = 0.5*mp*p.r^2;   % pulley inertia
p.mpd = 0.05;  % Piston and diaphragm mass, kg
p.mw = 0.1;   % Total mass of the water
p.mw2 = p.mw/2;   % Mass of half the water
p.mball = 0.058; % tennis ball weighs 58g

rad_rod = 0.01; % radius of rod: 1 cm
p.l_rod = 10*p.r; % length of rod is 10x radius of pulley
vol_rod = pi*(rad_rod^2-(rad_rod-0.002)^2)*p.l_rod; % pi*(r_outer^2 - r_inner^2)*length
p.mrod = vol_rod*2700;
p.Irod = p.mrod*p.l_rod^2/3;

% Stiffnesses
p.kp = 2014000; % Stiffness of the belt N/m 
p.kh = 1573;   % Stiffness of the hose N/m of y1
p.kball = 16600;   % Stiffness of tennis ball - 95 lb/in = 16637 N/m

% Damping
p.bp = 100;     % Damping of the belt
p.bf = 2.137;     % Viscous friction N/(m/s) of y1

% Torque curve
p.freq = 8;
p.ampli=.75;

% External disturbance
% p.dist_amp = .5; % Amplitude of disturbance: ~30 degrees
% p.dist_freq = 0.25; % Frequency of disturbance, Hz

% traj_fun = @(t) disturbanceTrajectory(p.dist_amp, p.dist_freq, t);

% % Set up controller
% c.Kp = 5000;
% c.Kd =30;
% c.k = 1.5;
% tau_des = 0;

% set up controller
% ctlr_fun = @(t,X) ctlrRDHTforce(t,X,c,p,tau_des, traj_fun);

%% Simulate the system
X0 = [.5 0 .5*p.r 0 .5*p.r 0 .5 0, 1.5, 0];
ctlr_fun = @(t,X,freq) ctlrRDHTBallBouncing(t,X,p);
[t_vec, X_vec] = simRDHTballBounce(X0,p,ctlr_fun);


%% retrieve control force
% force = ctlr_fun(t_vec',X_vec');
% figure
% plot(t_vec, force)
% xlabel('Time (s)')
% ylabel('Controller Force (Nm)')
% title('Controller Output')

%% Retrieve output forces
% [~, ~, ddy] = traj_fun(t_vec);
% system_state = X_vec(1:8,:);
% tau_out = [0  0   0   0   p.kp*p.r    p.bp*p.r   -p.kp*p.r^2   -p.bp*p.r^2]*system_state;
% % output_tau = calculate_output_torque(t_vec, X_vec, traj_fun)
% figure
% plot(t_vec, tau_out,  'b-', 'LineWidth', 2)
% ylabel('Output Force (Nm)')
% xlabel('Time (s)')
% title('Torque on Output')


%% Plotting
figure
% plot(t_vec, X_vec(1,:))
plot(t_vec, X_vec(1,:));
hold on
plot(t_vec, X_vec(7,:))
% plot(t_vec, 5*sin(t_vec))
legend('Theta1', 'Theta2')
xlabel('Time (s)')
ylabel('Radians')
title('Angular Displacement')

%% Plot Bouncy Ball
figure
plot(t_vec, X_vec(9,:));
hold on
plot(t_vec, p.h+7.5*p.r*sin(X_vec(7,:)));
xlabel('Time (s)')
ylabel('Position (m)')
title('Bouncy Ball Position')

for i=1:length(t_vec)
y(i) = p.ampli*sin(p.freq*2*pi.*t_vec(i));    
end

figure

plot(t_vec,y,'r')
hold on
plot(t_vec, X_vec(7,:),'b')
title('Desired vs actual')
legend('desired pose','actual pose')

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

% % % % Animate
exportVideo = false;
playbackRate = 1;
RDHTAnimation_ball(p,t_vec,X_vec,exportVideo,playbackRate);

