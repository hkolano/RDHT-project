%{
ROB 542: Actuator Dyamics, Assignment 5 & 6
Rolling Diaphragm Hydrostatic Transmission simulation
Main code
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
p.khose = 1573;   % Stiffness of the hose N/m of y1
p.kacc = 1; % Stiffness of accumulator, complete guess
% p.kh = 1/(1/p.khose+1/p.kacc); % Effective stiffness of "hose"
p.kh = p.khose;

% Damping
p.bp = 200;     % Damping of the belt
p.bf = 2.137;     % Viscous friction N/(m/s) of y1

% Set up controller
c.k = 1.4;
tau_des = 1;

% set up controller
% ctlr_fun = @(t,t_last,X,X_last) ctlrRDHTforce(t,t_last,X,X_last,c,p,tau_des);
ctlr_fun = @(t,X) ctlrRDHTforce2(t,X,c,p,tau_des);


frequencies = [0.2 0.4 0.6 0.8 1 1.2 1.4 2 3 4 5 6 8 9 10];
% frequencies = [0.8]
rmserrors = zeros(length(frequencies),1);
peak2peaks = zeros(length(frequencies),1);

for i = 1:length(frequencies)
    
    % External disturbance
    p.dist_amp = .5; % Amplitude of disturbance: ~30 degrees
    p.dist_freq = frequencies(i);%0.25; % Frequency of disturbance, Hz

    traj_fun = @(t) disturbanceTrajectory(p.dist_amp, p.dist_freq, t);

%% Simulate the system
X0 = [0 0 0 0 0 0 0 0];

[t_vec, X_vec] = simRDHTforceControl(X0,p, traj_fun, ctlr_fun);

tau_out = [0  0  0    0   p.kp*p.r    p.bp*p.r    -p.kp*p.r^2     -p.bp*p.r^2]*X_vec';
    rmserrors(i) = rms(tau_out(100:end)-tau_des);
    peak2peaks(i) = peak2peak(tau_out(floor(length(tau_out)/4):end)/2);
    
%     plot_output_force(t_vec,X_vec,p,c,tau_des)
       
end

figure
semilogx(frequencies, rmserrors)
xlabel('Frequency (Hz)')
ylabel('RMS error')
title('RMS error of Force control')

figure
semilogx(frequencies, peak2peaks)
xlabel('Frequency (Hz)')
ylabel('Output Torque Error Amplitude (Nm)')
title('Output Torque Error Amplitude')

%% Plotting (functions)

plot_output_force(t_vec,X_vec,p,c,tau_des)


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
% 
% % figure
% % plot(t_vec, X_vec(:,2));
% % legend('D Theta 1')
% 
% figure
% plot(t_vec, X_vec(:,3))
% hold on
% plot(t_vec, X_vec(:,5))
% legend('x1', 'x2')
% xlabel('Time (s)')
% ylabel('Displacement (m)')
% title('Piston Displacement')

%Animate
% exportVideo = false;
% playbackRate = 1;
% RDHTAnimation(p,t_vec,X_vec,exportVideo,playbackRate);

function plot_output_force(t_vec,X_vec,p,c, tau_des)
    tau_out = [0  0  0    0   p.kp*p.r    p.bp*p.r    -p.kp*p.r^2     -p.bp*p.r^2]*X_vec';
%     force_out = tau_out/(p.l_rod);
    F = -c.k*(tau_out-tau_des)+tau_des;
    figure
    plot(t_vec, tau_out)
    hold on
    plot(t_vec, ones(length(t_vec),1)*tau_des)
%     plot(t_vec, F)
    rms(tau_out(100:end)-tau_des)
    ylabel('Output Force (Nm)')
    xlabel('Time (s)')
    title('Force on Output')
    legend('Actual output torque', 'Desired output torque') %, 'Control Torque')
end