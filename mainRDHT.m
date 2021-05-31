%{
ROB 542: Actuator Dyamics, Assignment 5 & 6

Rolling Diaphragm Hydrostatic Transmission simulation

Main code
Last modified by Hannah Kolano 5/20/21
%}

clear all

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

%% Simulate the system
X0 = [0 0 0 0 0 0 0 0];

[t_vec, X_vec] = simRDHT(X0,p);

%%  Sweep
% frequencies = [0.1 0.2 0.5 1 2 5 10 15 20 22.5 25 27.5 30 35 40 45 50 80 100];
frequencies = [8];
n = length(frequencies);
ratios = [];

for i=1:n
    p.freq=frequencies(i)*2*pi;
    [t_vec, X_vec] = simRDHT(X0,p);

   input_pulley_pos = X_vec(:,1);
   output_pulley_pos = X_vec(:,7);
   
   input_amp = peak2peak(input_pulley_pos);
   output_amp = peak2peak(output_pulley_pos);
   ratio = output_amp/input_amp
   ratios(i) = ratio;
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
end

%% Plot Bode
db_ratios = mag2db(ratios);
figure
semilogx(frequencies, db_ratios, 'LineWidth', 2)
xlabel('Frequency (Hz)')
ylabel('Amplitude Ratio (dB)')
title('Output:Input Ratio')

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
% plot(t_vec, X_vec(:,2));
% legend('D Theta 1')

figure
plot(t_vec, X_vec(:,3))
hold on
plot(t_vec, X_vec(:,5))
legend('x1', 'x2')
xlabel('Time (s)')
ylabel('Displacement (m)')
title('Piston Displacement')
%
% figure
% plot(t_vec, X_vec(:,4))
% legend('dx1')

figure

plot(t_vec, X_vec(:,7)-X_vec(:,1))
xlabel('Time (s)')
ylabel('Position error (m)')
title('Input-Output shaft')

%Animate
% exportVideo = false;
% playbackRate = 1;
% RDHTAnimation(p,t_vec,X_vec,exportVideo,playbackRate);
