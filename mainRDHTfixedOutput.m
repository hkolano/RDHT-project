%{
ROB 542: Actuator Dyamics, Assignment 5 & 6

Rolling Diaphragm Hydrostatic Transmission simulation

Runs with fixed ouput sim function 
Last modified by Hannah Kolano 5/20/21
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

% Sinusoidal input
p.tamp = 0.5;
p.freq = 2*pi;

%% Simulate the system
X0 = [0 0 0 0 0 0 0 0];

[t_vec, X_vec] = simRDHTfixedOutput(X0,p);

%% Plotting

f_on_output =  [0  0  0  0  p.kp*p.r/p.Ip   p.bp*p.r/p.Ip   -p.kp*p.r^2/p.Ip   -p.bp*p.r^2/p.Ip]*X_vec.';

figure
plot(t_vec, p.tamp*cos(p.freq.*t_vec))
hold on
plot(t_vec, f_on_output*p.Ip)
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Input', 'Output')

% plot_angles(t_vec, X_vec(:,1), X_vec(:,7));
% 
% plot_angle_error(t_vec, X_vec(:,1), X_vec(:,7));
% 
% plot_pistons(t_vec, X_vec(:,3), X_vec(:,5));

%Animate
% exportVideo = false;
% playbackRate = 1;
% RDHTAnimation(p,t_vec,X_vec,exportVideo,playbackRate);

%% Plotting functions
function plot_angles(t, ang1, ang2)
    figure
    plot(t, ang1);
    hold on
    plot(t, ang2)
    legend('Theta1', 'Theta2')
    xlabel('Time (s)')
    ylabel('Radians')
    title('Angular Displacement')
end

function plot_pistons(t, pist1, pist2)
    figure
    plot(t, pist1)
    hold on
    plot(t, pist2)
    legend('x1', 'x2')
    xlabel('Time (s)')
    ylabel('Displacement (m)')
    title('Piston Displacement')
end

function plot_angle_error(t, ang1, ang2)
    figure
    plot(t, ang2-ang1)
    xlabel('Time (s)')
    ylabel('Angular error (rad)')
    title('Input-Output shaft')
end
