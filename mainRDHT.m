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
p.r = 12.5/1000; % radius of the pulley, in m
p.A1 = 0.0005; % Area of input piston, in m^2 (this is a little under a square inch)
p.A2 = 0.0005; % Area of ouput piston, in m^2
p.a = 0.00005; % Area of the tube, in m^2

% Masses
mp = 0.05;  % Pulley mass
p.Ip = 0.5*mp*p.r^2;   % pulley inertia
p.mpd = 0.05;  % Piston and diaphragm mass, kg
p.mw = 0.1;   % Total mass of the water
p.mw2 = p.mw/2;   % Mass of half the water

% Stiffnesses
p.kp = 100; % Stiffness of the belt 100 MPa
p.kh = 2;   % Stiffness of the hose (UNITS???)

% Damping
p.bp = 0.1;     % Damping of the belt
p.bf = 0.6;     % Viscous friction

%% Simulate the system
X0 = [0 0 0 0 0 0 0 0];

[t_vec, X_vec, ~, mask] = simRDHT(X0,p);

%% Plotting
figure
plot(t_vec, X_vec(1,:))
hold on
plot(t_vec, X_vec(3,:))
% % plot(t_vec, 5*sin(t_vec))
% legend('Theta1', 'Theta2', 'Force input')
%
% figure
% plot(t_vec, X_vec(3,:))
% hold on
% plot(t_vec, X_vec(5,:))
% legend('x1', 'x2')
%
% %Animate the mass
% exportVideo = false;
% playbackRate = .5;
% RDHTAnimation(p,t_vec,X_vec,exportVideo,playbackRate);
