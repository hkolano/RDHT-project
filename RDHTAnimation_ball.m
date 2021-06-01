function RDHTAnimation(p,t,X,exportVideo,playbackRate)
% For SE3

FPS=60;
% addpath(fullfile(pwd,'..', 'groupTheory'))
% For CubeClass and SpringClass
% addpath(fullfile(pwd,'..', 'visualization'))

%% pulley dimensions

% Tennis ball dimensions
ball_r = 0.0675/2;
% Pulley
pulley_R = .025; % pulley are A and B , R-radius, C center co ordinates
pulley_Cy=p.h+ball_r; % Height of pulley off of ground
pulley_aCx=0;
pulley_bCx=.5;

%% Reference Piston dimensions
% Reference geometry for diaphragm ellipse
piston_w = .05; %width
piston_h = .125; %height

% Piston reference geometry center positions
piston_Cy = p.h+.15; % Height of center of piston

piston_LaCx=pulley_aCx-pulley_R; % A and B are input pistons - L (left)
piston_LbCx=pulley_aCx+pulley_R;

piston_RaCx=pulley_bCx-pulley_R; % A and B are output pistons - R (left)
piston_RbCx=pulley_bCx+pulley_R;

%% Actual Piston dimensions
% inner pistons are A and B (two in left and 2 in right) , R-radius, C center co ordinates
pistonIn_w = .025;
pistonIn_h = .125;

% Initial heights
pistonInner_LaCy=piston_Cy;
pistonInner_LbCy=piston_Cy;
pistonInner_RaCy=piston_Cy;
pistonInner_RbCy=piston_Cy;


%% diaphragm dimentions
diaphragm_LBaseH=0;
diaphragm_RBaseH=0;

%%% set up objects
piston_La =CubeClass([piston_w,piston_h]);
piston_Lb =CubeClass([piston_w,piston_h]);
piston_Ra =CubeClass([piston_w,piston_h]);
piston_Rb =CubeClass([piston_w,piston_h]);
pulley_aObj = SphereClass(pulley_R);
pulley_bObj = SphereClass(pulley_R);
pistonInner_La =CubeClass_A([pistonIn_w,pistonIn_h]);
pistonInner_Lb =CubeClass_A([pistonIn_w,pistonIn_h]);
pistonInner_Ra =CubeClass_A([pistonIn_w,pistonIn_h]);
pistonInner_Rb =CubeClass_A([pistonIn_w,pistonIn_h]);

ballObj = SphereClass(ball_r);
% Create a figure handle
h.figure = figure;

%This sets figure dimension which will dictate video dimensions
h.figure.Position(3:4) = [1280 720];
movegui(h.figure)

%% set up  plot

pistonInner_La.plot
pistonInner_Lb.plot
pistonInner_Ra.plot
pistonInner_Rb.plot
ballObj.plot


%%% ploting the belt around the pulley that appears stationary
ph=0:.1:pi;
for i=1:length(ph)
beltAx(i)=pulley_aCx+pulley_R*cos(-ph(i));
beltAy(i)=pulley_Cy+pulley_R*sin(-ph(i));
beltBx(i)=pulley_bCx+pulley_R*cos(-ph(i));
beltBy(i)=pulley_Cy+pulley_R*sin(-ph(i));
end

plot(beltAx,beltAy,'r','LineWidth',4)
plot(beltBx,beltBy,'r','LineWidth',4)
plot([-.22 1],[-ball_r -ball_r],'color',[.45 0 .08],'LineWidth',6)

%%% stationary like to fix the plot
plot([-.02 -.02],[-.02 1],'color',[1 1 1]);
plot([-.02 1],[-.02 -.02],'color',[1 1 1]);
%%%%% the pulley plot which is  stationary
ph=0:.1:2*pi;
for i=1:length(ph)
    pulleyLx(i)=pulley_aCx+pulley_R*cos(ph(i));
    pulleyLy(i)=pulley_Cy+pulley_R*sin(ph(i));
    pulleyRx(i)=pulley_bCx+pulley_R*cos(ph(i));
    pulleyRy(i)=pulley_Cy+pulley_R*sin(ph(i));


end
    fill(pulleyLx,pulleyLy,[1,1,.5]);
    fill(pulleyRx,pulleyRy,[1,1,.5]);

view(2)
title('Simulation')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')

% h.figure.Children(1).DataAspectRatioMode = 'manual';
h.figure.Children(1).DataAspectRatio = [1 1 1];

if exportVideo
   v = VideoWriter('puckAnimation.mp4', 'MPEG-4');
   v.FrameRate = FPS;
   open(v)
end
tic;

flag=0;
for t_plt = t(1):playbackRate*1.0/FPS:t(end)

    if flag==1
        delete(plt) % reset 2D plots
    else flag==0
         flag =1;
    end

     %% input from the main code
    x_state = interp1(t',X',t_plt);
    pistonInner_LaCy=x_state(3)+piston_Cy;
    pistonInner_LbCy=-x_state(3)+piston_Cy;
    pistonInner_RaCy=x_state(5)+piston_Cy;
    pistonInner_RbCy=-x_state(5)+piston_Cy;
    ball_pose=x_state(9);
    input=-x_state(1);
    output=-x_state(7);


    %% resets

    pistonInner_La.resetFrame
    pistonInner_Lb.resetFrame
    pistonInner_Ra.resetFrame
    pistonInner_Rb.resetFrame
    ballObj.resetFrame



    %% pistons

    ballObj.globalMove(SE3([pulley_bCx+7.5*pulley_R, ball_pose, 0]));

    piston_La.globalMove(SE3([piston_LaCx, piston_Cy, 0]))
    piston_Lb.globalMove(SE3([piston_LbCx, piston_Cy, 0]))
    piston_Ra.globalMove(SE3([piston_RaCx, piston_Cy 0]))
    piston_Rb.globalMove(SE3([piston_RbCx, piston_Cy 0]))


%% plot the pulley
    ph=0:.1:pi;
    for i=1:length(ph)
      pulley_ax(i)=pulley_aCx+pulley_R*cos(ph(i));
      pulley_ay(i)=pulley_Cy+pulley_R*sin(ph(i));

    end

    fill(pulley_ax,pulley_ay,[1,1,.5])
    %% inner piston that moves
    pistonInner_La.globalMove(SE3([piston_LaCx, pistonInner_LaCy, 0]))
    pistonInner_Lb.globalMove(SE3([piston_LbCx, pistonInner_LbCy, 0]))
    pistonInner_Ra.globalMove(SE3([piston_RaCx, pistonInner_RaCy, 0]))
    pistonInner_Rb.globalMove(SE3([piston_RbCx, pistonInner_RbCy, 0]))

    %% ploting the belt from pulley to the inner piston
    plt(1)=plot([pulley_aCx+pulley_R*cos(-pi) piston_LaCx],[pulley_Cy+pulley_R*sin(-pi) pistonInner_LaCy-pistonIn_h*.5],'r','LineWidth',4);
    plt(2)=plot([pulley_aCx+pulley_R*cos(0) piston_LbCx],[pulley_Cy+pulley_R*sin(0) pistonInner_LbCy-pistonIn_h*.5],'r','LineWidth',4);
    plt(3)=plot([pulley_bCx+pulley_R*cos(-pi) piston_RaCx],[pulley_Cy+pulley_R*sin(-pi) pistonInner_RaCy-pistonIn_h*.5],'r','LineWidth',4);
    plt(4)=plot([pulley_bCx+pulley_R*cos(0) piston_RbCx],[pulley_Cy+pulley_R*sin(0) pistonInner_RbCy-pistonIn_h*.5],'r','LineWidth',4);

    %% ploting the diaphragm
    ph=0:.1:pi;
    for i=1:length(ph)
        diaALx(i)=piston_LaCx+piston_w*.5*cos(ph(i));
        diaALy(i)=(piston_Cy+(piston_h*.5)-diaphragm_LBaseH)+(pistonInner_LaCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)-diaphragm_LBaseH))*sin(ph(i));
        diaBLx(i)=piston_LbCx+piston_w*.5*cos(ph(i));
        diaBLy(i)=(piston_Cy+(piston_h*.5)-diaphragm_LBaseH)+(pistonInner_LbCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)-diaphragm_LBaseH))*sin(ph(i));
        diaARx(i)=piston_RaCx+piston_w*.5*cos(ph(i));
        diaARy(i)=(piston_Cy+(piston_h*.5)-diaphragm_RBaseH)+(pistonInner_RaCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)-diaphragm_RBaseH))*sin(ph(i));
        diaBRx(i)=piston_RbCx+piston_w*.5*cos(ph(i));
        diaBRy(i)=(piston_Cy+(piston_h*.5)-diaphragm_RBaseH)+(pistonInner_RbCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)-diaphragm_RBaseH))*sin(ph(i));
    end
   plt(5)= fill(diaALx,diaALy,[1,0,1]);
   plt(6)= fill(diaBLx,diaBLy,[1,0,1]);
   plt(7)= fill(diaARx,diaARy,[1,0,1]);


   plt(8)= fill(diaBRx,diaBRy,[1,0,1]);
   
    %%% ploting the water tubes
    water_ALx=piston_LaCx+piston_w*.5*cos(pi/2);
    water_ALy=(piston_Cy+(piston_h*.5))+(pistonInner_LaCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)))*sin(pi/2);
    water_BLx=piston_LbCx+piston_w*.5*cos(pi/2);
    water_BLy=(piston_Cy+(piston_h*.5))+(pistonInner_LbCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)))*sin(pi/2);


    water_ARx=piston_RaCx+piston_w*.5*cos(pi/2);
    water_ARy=(piston_Cy+(piston_h*.5))+(pistonInner_RaCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)))*sin(pi/2);
    water_BRx=piston_RbCx+piston_w*.5*cos(pi/2);
    water_BRy=(piston_Cy+(piston_h*.5))+(pistonInner_RbCy+(pistonIn_h*.5)-(piston_Cy+(piston_h*.5)))*sin(pi/2);



    plt(9)= plot([water_ALx water_ALx],[water_ALy piston_Cy+.5],'b','LineWidth',4);
    plt(10)= plot([water_ALx water_BRx],[piston_Cy+.5 piston_Cy+.5],'b','LineWidth',4);
    plt(11)= plot([water_BRx water_BRx],[piston_Cy+.5 water_BRy],'b','LineWidth',4);

    plt(12)= plot([water_BLx water_BLx],[water_BLy piston_Cy+.25],'b','LineWidth',4);
    plt(13)= plot([water_BLx water_ARx],[piston_Cy+.25 piston_Cy+.25],'b','LineWidth',4);
    plt(14)= plot([water_ARx water_ARx],[piston_Cy+.25 water_ARy],'b','LineWidth',4);
    
    plt(15)=plot([pulley_aCx pulley_aCx+pulley_R*cos(input)],[pulley_Cy pulley_Cy+pulley_R*sin(input)],'b','LineWidth',4);
    plt(20)=plot([pulley_bCx pulley_bCx+10*pulley_R*cos(output)],[pulley_Cy pulley_Cy+10*pulley_R*sin(output)],'color',[.5 .5 .5],'LineWidth',2);
    

%
%     piston_La.updatePlotData
%     piston_Lb.updatePlotData
%     piston_Ra.updatePlotData
%     piston_Rb.updatePlotData

%     pulley_aObj.updatePlotData
%     pulley_bObj.updatePlotData

    pistonInner_La.updatePlotData
    pistonInner_Lb.updatePlotData
    pistonInner_Ra.updatePlotData
    pistonInner_Rb.updatePlotData
    ballObj.updatePlotData



    if exportVideo %Draw as fast as possible for video export
    drawnow
    frame = getframe(h.figure);
    writeVideo(v,frame);
    else % pause until 1/FPS of a second has passed then draw
        while( toc < 1.0/FPS)
            pause(0.002)
        end
     drawnow
     tic;

end

end %RDHTAnimation
