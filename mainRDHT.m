%{
ROB 542: Actuator Dyamics, Assignment 5

Rolling Diaphragm Hydrostatic Transmission simulation

Main code
Last modified by Hannah Kolano 5/13/21
%}

close all

%% Set up parameters

% stroke: 56.8mm
% cylinders: 25bar max
% Direct drive motor peak torque 19.6 Nm

% Geometry
p.r = 23.9/1000; % radius of the pulley, in mm (from Design and Experiment...)
p.A1 = (0.024/2)^2*pi; % Area of input piston, in m^2 (this is a little under a square inch)
p.A2 = (0.024/2)^2*pi; % Area of ouput piston, in m^2
p.a = (.006/2)^2*pi; % Area of the tube, in m^2

% Masses
mp = 0.05;  % Pulley mass
p.Ip = 0.5*mp*p.r^2;   % pulley inertia
p.mpd = 0.05;  % Piston and diaphragm mass, kg
p.mw = 0.1;   % Total mass of the water
p.mw2 = p.mw/2;   % Mass of half the water

% Stiffnesses
p.kp = 10000; % Stiffness of the belt N/m (from paper sources)
p.kh = 5;   % Stiffness of the hose (UNITS???)

% Damping
p.bp = .001;     % Damping of the belt 
p.bf = 1; %547;     % Viscous friction

%% Simulate the system
X0 = [0 0 0 0 0 0 0 0];

[t_vec, X_vec] = simRDHT(X0,p);

%% Plotting
figure
% plot(t_vec, X_vec(1,:))
plot(t_vec, X_vec(:,1));
hold on
plot(t_vec, X_vec(:,7))
% plot(t_vec, 5*sin(t_vec))
legend('Theta1', 'Theta2')
title('Angular Displacement')

% figure
% plot(t_vec, X_vec(:,2));
% legend('D Theta 1')

figure
plot(t_vec, X_vec(:,3))
hold on
plot(t_vec, X_vec(:,5))
legend('x1', 'x2')
title('Piston Displacement')
% 
% figure
% plot(t_vec, X_vec(:,4))
% legend('dx1')

%Animate the mass
% exportVideo = false;
% playbackRate = .5;
% RDHTAnimation(p,t_vec,X_vec,exportVideo,playbackRate);