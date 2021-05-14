%{
ROB 542: Actuator Dyamics, Assignment 5

Rolling Diaphragm Hydrostatic Transmission simulation

Main code
Last modified by Hannah Kolano 5/13/21
%}

close all

%% Set up parameters

% Geometry
p.r = 0.05; % radius of the pulley, in m
p.A1 = 0.0005; % Area of input piston, in m^2 (this is a little under a square inch)
p.A2 = 0.0005; % Area of ouput piston, in m^2
p.a = 0.0001; % Area of the tube, in m^2

% Masses
p.Ip = 0.05;   % pulley inertia
p.mpd = 0.05;  % Piston and diaphragm mass, kg
p.mw = 0.1;   % Total mass of the water
p.mw2 = p.mw/2;   % Mass of half the water

% Stiffnesses
p.kp = 1; % Stiffness of the belt (UNITS???)
p.kh = 1;   % Stiffness of the hose (UNITS???)

% Damping
p.bp = 1;     % Damping of the belt 
p.bf = 1;     % Viscous friction

%% Simulate the system
X0 = [0 0 0 0 0 0 0 0];

[t_vec, X_vec, ~, mask] = simRDHT(X0,p);