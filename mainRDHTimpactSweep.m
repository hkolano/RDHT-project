%{
ROB 542: Actuator Dyamics, Assignment 7

Rolling Diaphragm Hydrostatic Transmission simulation

IMPACT CONTROL 

Main code
Last modified by Hannah Kolano 6/1/21
%}

close all

%% Set up parameters

% cylinders: 25bar max
% Direct drive motor peak torque 19.6 Nm
p.rball = 0.0675/2;

% Geometry
p.r = 23.9/1000; % radius of the pulley, in mm (from Design and Experiment...)
p.A1 = (0.024/2)^2*pi; % Area of input piston, in m^2 (this is a little under a square inch)
p.A2 = (0.024/2)^2*pi; % Area of ouput piston, in m^2
p.a = (.006/2)^2*pi; % Area of the tube, in m^2
p.strokelim = 56.8/2/1000; % Stroke length limit (either way from 0)

p.h = 0; % Distance between pulley center and ground
p.obstacle_height = -p.rball; % object will contact rod when rod contact position is at this height

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
p.Imotor = 0; %9.04*10^-5;

% Stiffnesses
p.kp = 2014000; % Stiffness of the belt N/m 
p.khose = 1573;   % Stiffness of the hose N/m of y1
p.kacc = 1000; % Stiffness of accumulator, complete guess
p.kh = 1/(1/p.khose+1/p.kacc); % Effective stiffness of "hose"
% p.kh = p.khose;

% Damping
p.bp = 200;     % Damping of the belt
p.bf = 2.137;     % Viscous friction N/(m/s) of y1

% Torque curve
p.ampli=1;
p.des_theta = 1;

% frequencies = [0.1 0.2 0.5 1 2 3 4 5 6 7 8 9 10 12 14 16 18 20];
frequencies = [0.24, 1.02, 2.32, 15];
impulses = zeros(length(frequencies),1);
pressures= {};
time_vecs = {};
vels = zeros(length(frequencies),1);

for f = 1:length(frequencies)

p.freq = frequencies(f);
% p.freq = 1;

% set up controller
ctlr_fun = @(t,X)ctlrRDHTPosition(t,X,p);

%% Simulate the system
X0 = [-1 0 -1*p.r 0 -1*p.r 0 -1 0];

[t_vec, X_vec] = simRDHTimpact(X0,p, ctlr_fun);

P2 = plot_pressure(t_vec, X_vec, p);

impulses(f) = find_max_force(t_vec,X_vec, p);
pressures{f} = P2;
time_vecs{f} = t_vec;
vels(f) = max(X_vec(8,:))

end % Frequency sweep
close all

%% plot
figure
plot(time_vecs{1}, pressures{1}/1000000)
hold on
for i = 2:length(frequencies)
    plot(time_vecs{i}, pressures{i}/1000000)
end
grid on
    xlabel('Frequency (Hz)')
    ylabel('Pressure (Mpa)')
    title('Pressure in Lines (with Accumulator = 1000N/m)')
    legend('w=1.5', 'w=6.5', 'w=15.0', 'w=27.0')

%% plot
% bode_plot(frequencies, impulses)

% %% Plot Angles
% plot_input_output_angles(t_vec,X_vec);
% 
% plot_obstacle_and_rod(t_vec, X_vec, p);
% 
% %% Plot Force
% plot_output_force(t_vec,X_vec,p);
% P2 = plot_pressure(t_vec, X_vec, p);
% 
% %% Plot desired angle
% plot_desired_output_angle(t_vec,X_vec,p);

%% Animate
% exportVideo = false;
% playbackRate = 1;
% RDHTAnimationImpact(p,t_vec,X_vec,exportVideo,playbackRate);

%% Plotting Functions

function bode_plot(x_vals, y_vals)
   figure;
    plot(x_vals, y_vals)
    xlabel('Frequency (Hz)')
    ylabel('Maximum Force imparted (N)')
    title('Bode Plot: Maximum Forces')
end

function plot_desired_output_angle(t_vec,X_vec, p)
y = zeros(length(t_vec));
for i = 1:length(t_vec)
    t = t_vec(i);
    if t < 1/(2*p.freq)
        y(i) = p.ampli*-cos(p.freq*2*pi*t);
    else
        y(i) = p.des_theta;
    end
end
figure
plot(t_vec, y(:,1))
hold on
plot(t_vec, X_vec(7,:))
xlabel('Time (s)')
ylabel('Angle')
legend('Desired Angle', 'Actual Theta2')
end

function force_max = find_max_force(t_vec,X_vec,p)
    system_state = X_vec(1:8,:);
    tau_out = [0  0  0    0   p.kp*p.r    p.bp*p.r    -p.kp*p.r^2     -p.bp*p.r^2]*system_state;
    force_out = tau_out/(p.l_rod);
%     disp('Max impulse force:')
    force_max = max(force_out);
end

function plot_output_force(t_vec,X_vec,p)
    system_state = X_vec(1:8,:);
    tau_out = [0  0  0    0   p.kp*p.r    p.bp*p.r    -p.kp*p.r^2     -p.bp*p.r^2]*system_state;
    force_out = tau_out/(p.l_rod);
    disp('Max impulse force:')
    disp(max(force_out))
    figure
    plot(t_vec, force_out,  'm-')
    ylabel('Output Force (N)')
    xlabel('Time (s)')
    title('Force on Obstacle')
%     figure
%     plot(t_vec, (tau_out/p.r)/p.A2)
%     ylabel('Pressure difference est?')
end

function P2 = plot_pressure(t_vec, X_vec,p)
%     F2 = [0 0 0 0 p.kp  p.bp    -p.kp*p.r   -p.bp*p.r]*X_vec(1:8,:);
%     P2 = F2/p.A2;
%     F1 = [-p.kp -p.bp    p.kp*p.r    p.bp*p.r    0   0   0   0]*X_vec(1:8,:);
%     P1 = F1/p.A1;

%     dt = t_vec(2)-t_vec(1);
%     y1_acc = zeros(length(t_vec),1);
%     y2_acc = zeros(length(t_vec),1);
%     for i = 1:length(t_vec)-1
%         x2_acc = (X_vec(4,i+1) - X_vec(4,i))/dt;
%         y2_acc(i+1) = x2_acc*p.A2/p.a;
%     end
    F2 = p.A2^2/(p.a^2)*[0 0 p.kh 0 -p.kh -p.bf 0 0]*X_vec(1:8,:);
%     F2_wacc = p.A2^2/(p.a^2)*([0 0 p.kh 0 -p.kh -p.bf 0 0]*X_vec(1:8,:) - p.mw2*y2_acc');
    P2 = F2/p.a;
%     P2_wacc = F2_wacc/p.a;
    F1 = p.A1^2/(p.a^2)*[0 0 -p.kh -p.bf p.kh 0 0 0]*X_vec(1:8,:);
    P1 = F1/p.a;
    
    disp('Max pressure (MPa):')
    disp(max(P2)/1000000);
    
    figure
    plot(t_vec, P2/1000000)
    hold on
    grid on
%     plot(t_vec, P1)
    xlabel('Time (s)')
    ylabel('Pressure (MPa)')
    title('Pressure in Lines')
    ylim([0 50])
    
%     legend('P2')
   
end

function plot_obstacle_and_rod(t_vec, X_vec, p)
    figure
    plot(t_vec, ones(length(X_vec))*p.obstacle_height+p.rball, 'm-');
    hold on
    plot(t_vec, p.h-p.l_rod*sin(X_vec(7,:)), 'b-');
    xlabel('Time (s)')
    ylabel('Position (m)')
    title('Rod vs Obstacle')
end

function plot_input_output_angles(t_vec, X_vec)
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
    
    figure
    plot(t_vec, X_vec(8,:))
    ylabel('Angular Velocity')
end

