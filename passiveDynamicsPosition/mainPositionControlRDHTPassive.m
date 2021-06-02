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
freq=1;
% Masses
mp = 0.05;  % Pulley mass
p.Ip = 0.5*mp*p.r^2;   % pulley inertia
p.mpd = 0.05;  % Piston and diaphragm mass, kg
p.mw = 0.1;   % Total mass of the water
p.mw2 = p.mw/2;   % Mass of half the water

% Stiffnesses
p.kp = 2014000; % Stiffness of the belt N/m 
p.k_horse = 1573;% Stiffness of the hose N/m of y1
p.k_accu = 100;
p.kh=(p.k_horse*p.k_accu)/(p.k_horse+p.k_accu);


% Damping
p.bp = 200;     % Damping of the belt
p.bf = 2.137;     % Viscous friction N/(m/s) of y1

%% Simulate the system
X0 = [0 0 0 0 0 0 0 0];
p.dist_amp = 4500; % Amplitude of disturbance: ~30 degrees
p.dist_freq = 5; % Frequency of disturbance, Hz
p.freq=1
traj_fun = @(t) disTrajPositionPassive(p.dist_amp, p.dist_freq, t);% External disturbance
% Set up controller
c.Kp = 10000;
c.Kd = 1000;
tau_des = .1;

% set up controller
ctlr_fun = @(t,X,freq) ctlrRDHTPositionPassive(t,X,freq);

[t_vec, X_vec] = simPositionControlPassiveRDHT(X0,p,c,freq, traj_fun, ctlr_fun);

%% Plotting
figure
% plot(t_vec, X_vec(1,:))
plot(t_vec, X_vec(:,1));
hold on
plot(t_vec, X_vec(:,7))
% plot(t_vec, 5*sin(t_vec))
legend('input pulley angle', 'output pulley angle')
xlabel('Time (s)')
ylabel('Radians')
title('Passive Dynamics Position control')

% figure
% plot(t_vec, X_vec(:,2));
% legend('D Theta 1')

figure
plot(t_vec, X_vec(:,3))
hold on
plot(t_vec, X_vec(:,5))
legend('input piston displacement', 'output piston displacement')
xlabel('Time (s)')
ylabel('Displacement (m)')
title('Passive Dynamics Position control')
%
% figure
% plot(t_vec, X_vec(:,4))
% legend('dx1')

figure

plot(t_vec, X_vec(:,7)-X_vec(:,1))
xlabel('Time (s)')
ylabel('Position error (rad)')
title('Input-Output shaft')
title('Passive Dynamics Position control')
 amp=.5;
 freq=1;
 
figure
dy = amp*freq*2*pi*cos(freq*2*pi.*t_vec);
plot(t_vec, dy)
hold on
plot(t_vec, X_vec(:,8))
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
legend('Desired velocity', 'Actual velocity')
title('Passive Dynamics Position control')

figure
y = amp*sin(freq*2*pi.*t_vec);
plot(t_vec, y)
hold on
plot(t_vec, X_vec(:,7))
xlabel('Time (s)')
ylabel('Position (rad)')
legend('Desired position', 'Actual position')
title('Passive Dynamics Position control')
% 
% exportVideo = 1;
% playbackRate = 1;
% RDHTAnimationPosConPassive(p,t_vec,X_vec,exportVideo,playbackRate);


for i=1:100
    freq=i;
     amp=.5;
    y = amp*sin(freq*2*pi.*t_vec);
    [t_vec, X_vec] = simPositionControlPassiveRDHT(X0,p,c,freq,traj_fun, ctlr_fun);
    fq(i)=i;
   output_amp = peak2peak(X_vec(:,7));
   input_amp = peak2peak(y);
   ratio(i) =output_amp/ input_amp;

    
   
end
figure
plot(fq,ratio,'md')
xlabel('Frequency (Hz)')
ylabel('Amplitude Ratio')
title('Passive dynamics frequency plot')
ylim([-2 5])



