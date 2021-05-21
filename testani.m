figure(1);
clf;
axis equal;
hold on
%Prep video writer
filename = 'RDHTDemo.mp4';
v = VideoWriter(filename,'MPEG-4');
v.FrameRate = 60;
open(v);

%Set up vector of joint angles to visualize
thetas = linspace(0,-pi/3,120);
thetas = [thetas,fliplr(thetas)];
%Guess at wrapping angle around bushings because math is hard
int_angle = pi/4;