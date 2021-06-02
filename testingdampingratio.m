p.r = 23.9/1000; % radius of the pulley, in mm (from Design and Experiment...)
p.A1 = (0.024/2)^2*pi; % Area of input piston, in m^2 (this is a little under a square inch)
p.A2 = (0.024/2)^2*pi; % Area of ouput piston, in m^2
p.a = (.006/2)^2*pi; % Area of the tube, in m^2
p.strokelim = 56.8/2/1000; % Stroke length limit (either way from 0)

p.h = 0.5; % Distance between pulley center and ground
p.obstacle_height = 0.37; % object will contact rod when rod contact position is at this height

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
p.khose = 1573;   % Stiffness of the hose N/m of y1
p.kacc = 750; % Stiffness of accumulator, complete guess
p.kh = 1/(1/p.khose+1/p.kacc); % Effective stiffness of "hose"
p.kh = p.khose;

% Damping
% p.bp = 100;     % Damping of the belt
p.bp = 2;
p.bf = 2.137;     % V
k = 0;
b = 0;

 M1 = p.mw2*p.A1/p.a + p.mpd*p.a/p.A1;
    M2 = p.mw2*p.A2/p.a + p.mpd*p.a/p.A2;
    kpaOVA1  = p.kp*p.a/p.A1;
    bpaOVA1  = p.bp*p.a/p.A1;
    kpaOVA2  = p.kp*p.a/p.A2;
    bpaOVA2  = p.bp*p.a/p.A2;


A = [0                 1                   0                           0                           0                           0                           0                   0; ...
        -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                           0                   0; ...
        0                   0                   0                           1                           0                           0                           0                   0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
        0                   0                   0                           0                           0                           1                           0                   0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
        0                   0                   0                           0                           0                           0                           0                   1; ...
        0                   0                   0                           0                           p.kp*p.r/(p.Ip+p.Irod)      p.bp*p.r/(p.Ip+p.Irod)      -p.kp*p.r^2/(p.Ip+p.Irod)     -p.bp*p.r^2/(p.Ip+p.Irod)];
[V,D] = eig(A);
diag(D)
figure
plot(D, 'md')
title('Small damping')

%% high
p.bp = 202.0;
A2 = [0                 1                   0                           0                           0                           0                           0                   0; ...
        -p.kp*p.r^2/p.Ip     -p.bp*p.r^2/p.Ip    p.kp*p.r/p.Ip             p.bp*p.r/p.Ip               0                           0                           0                   0; ...
        0                   0                   0                           1                           0                           0                           0                   0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
        0                   0                   0                           0                           0                           1                           0                   0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
        0                   0                   0                           0                           0                           0                           0                   1; ...
        0                   0                   0                           0                           p.kp*p.r/(p.Ip+p.Irod)      p.bp*p.r/(p.Ip+p.Irod)      -p.kp*p.r^2/(p.Ip+p.Irod)     -p.bp*p.r^2/(p.Ip+p.Irod)];
[V2,D2] = eig(A2);
D2diag = diag(D2)
sys = zpk([],[D2diag(1),D2diag(2)],1)
damp(sys)
% figure
% plot(D2, 'gd')
% title('Larger damping')
