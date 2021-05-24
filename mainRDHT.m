%{
ROB 542: Actuator Dyamics, Assignment 5

Rolling Diaphragm Hydrostatic Transmission simulation

Main code
Last modified by Hannah Kolano 5/13/21
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
p.w=1;
% Stiffnesses
p.kp = 10000; % Stiffness of the belt N/m (from paper sources)
p.kh = 1573;   % Stiffness of the hose N/m of y1

% Damping
p.bp = .5;     % Damping of the belt
p.bf = 2.137;     % Viscous friction N/(m/s) of y1

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
figure
hold on
%% bode plot
for i=1:50
p.w=i;
[t_vec, X_vec] = simRDHT(X0,p);

for i=1:length(t_vec)
f_out(i)=p.kp*p.r*X_vec(i,5)+p.bp*p.r*X_vec(i,6)-p.kp*p.r^2*X_vec(i,7)-p.bp*p.r^2*X_vec(i,8);
f_in(i)=-p.kp*p.r^2*X_vec(i,1)-p.bp*p.r^2*X_vec(i,2)+p.kp*p.r*X_vec(i,3)+p.bp*p.r*X_vec(i,4);

f(i)=f_out(i)/f_in(i);
end
g=mean(f);
plot(p.w,g,'ob')
ylim([-2 2])
title('bode plot')
end
%Animate
% exportVideo = false;
% playbackRate = 1;
% RDHTAnimation(p,t_vec,X_vec,exportVideo,playbackRate);
