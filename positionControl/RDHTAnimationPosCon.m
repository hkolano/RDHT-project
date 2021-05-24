function RDHTAnimationPosCon(p,t,X,exportVideo,playbackRate)
% For SE3

FPS=60;
addpath(fullfile(pwd,'..', 'groupTheory'))
% For CubeClass and SpringClass
addpath(fullfile(pwd,'..', 'visualization'))
%% piston for the wall
piston_wL = .5; %width
piston_hL = 1; %height

piston_LaCx=0; % A and B are input pistons - L (left)
piston_LaCy=3;
piston_LbCx=1;
piston_LbCy=3;

piston_RaCx=3; % A and B are output pistons - R (left)
piston_RaCy=3;
piston_RbCx=4;
piston_RbCy=3;

piston_wR = .5;
piston_hR = 1;

%%% pulley dimensions
pulley_aR = .5; % pulley are A and B , R-radius, C center co ordinates
pulley_aCx=.5;
pulley_aCy=2;
pulley_bR = .5;
pulley_bCx=3.5;
pulley_bCy=2;



% inner pistons are A and B (two in left and 2 in right) , R-radius, C center co ordinates

pistonIn_wL = .25;
pistonIn_hL = 1;
pistonInner_LaCx=0;
pistonInner_LaCy=3;
pistonInner_LaCy=3.1;
pistonInner_LbCx=1;
pistonInner_LbCy=3;

pistonInner_RaCx=3;
pistonInner_RaCy=3;
pistonInner_RbCx=4;
pistonInner_RbCy=3;
pistonIn_wR = .25;
pistonIn_hR = 1;

%% diaphragm dimentions
diaphragm_LBaseH=0;
diaphragm_RBaseH=0;

%%% set up objects
piston_La =CubeClass([piston_wL,piston_hL]);
piston_Lb =CubeClass([piston_wL,piston_hL]);
piston_Ra =CubeClass([piston_wR,piston_hR]);
piston_Rb =CubeClass([piston_wR,piston_hR]);
pulley_aObj = SphereClass(pulley_aR);
pulley_bObj = SphereClass(pulley_bR);
pistonInner_La =CubeClass_A([pistonIn_wL,pistonIn_hL]);
pistonInner_Lb =CubeClass_A([pistonIn_wL,pistonIn_hL]);
pistonInner_Ra =CubeClass_A([pistonIn_wR,pistonIn_hR]);
pistonInner_Rb =CubeClass_A([pistonIn_wR,pistonIn_hR]);


% Create a figure handle
h.figure = figure;

%This sets figure dimension which will dictate video dimensions
h.figure.Position(3:4) = [1280 720];
movegui(h.figure)

%% set up  plot
% piston_La.plot
% piston_Lb.plot
% piston_Ra.plot
% piston_Rb.plot
pistonInner_La.plot
pistonInner_Lb.plot
pistonInner_Ra.plot
pistonInner_Rb.plot
% pulley_aObj.plot
% pulley_bObj.plot

%%% ploting the belt around the pulley that appears stationary
ph=0:.1:pi;
for i=1:length(ph)
beltAx(i)=pulley_aCx+pulley_aR*cos(-ph(i));
beltAy(i)=pulley_aCy+pulley_aR*sin(-ph(i));
beltBx(i)=pulley_bCx+pulley_bR*cos(-ph(i));
beltBy(i)=pulley_bCy+pulley_bR*sin(-ph(i));
end

plot(beltAx,beltAy,'r','LineWidth',4)
plot(beltBx,beltBy,'r','LineWidth',4)

%%%%% the pulley plot which is  stationary
ph=0:.1:2*pi;
for i=1:length(ph)
    pulleyLx(i)=pulley_aCx+pulley_aR*cos(ph(i));
    pulleyLy(i)=pulley_aCy+pulley_aR*sin(ph(i));
    pulleyRx(i)=pulley_bCx+pulley_bR*cos(ph(i));
    pulleyRy(i)=pulley_bCy+pulley_bR*sin(ph(i));


end
    fill(pulleyLx,pulleyLy,[1,1,.5]);
    fill(pulleyRx,pulleyRy,[1,1,.5]);

view(2)
title('Simulation')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')

% h.figure.Children(1).DataAspectRatioMode = 'manual';
% h.figure.Children(1).DataAspectRatio = [1 1 1];

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
    x_state = interp1(t',X,t_plt);
    pistonInner_LaCy=x_state(3)*10+3
    pistonInner_LbCy=-x_state(3)*10+3
    pistonInner_RaCy=x_state(5)*10+3
    pistonInner_RbCy=-x_state(5)*10+3
    input=-x_state(1)
    output=-x_state(7)


%     x_state = interp1(t',X',t_plt);
%     pistonInner_LaCy=x_state(3)*10000+3
%     pistonInner_LbCy=-x_state(3)*10000+3
%     pistonInner_RaCy=x_state(5)*10000+3
%     pistonInner_RbCy=-x_state(5)*10000+3
%     pistonInner_LaCy=x_state(2)
%     pistonInner_RbCy=x_state(5)
    % Set axis limits (These will respect the aspect ratio set above)
%     h.figure.Children(1).XLim = [-1, 5];
%     h.figure.Children(1).YLim = [-1, 5];
%     h.figure.Children(1).ZLim = [-1.0, 1.0];

    %% resets
%     piston_La.resetFrame
%     piston_Lb.resetFrame
%     piston_Ra.resetFrame
%     piston_Rb.resetFrame
    pistonInner_La.resetFrame
    pistonInner_Lb.resetFrame
    pistonInner_Ra.resetFrame
    pistonInner_Rb.resetFrame
%     pulley_aObj.resetFrame
%     pulley_bObj.resetFrame
%


    %% pistons
%     piston_La.globalMove(SE3([piston_LaCx, piston_LaCy, 0]))
%     piston_Lb.globalMove(SE3([piston_LbCx, piston_LbCy, 0]))
%     piston_Ra.globalMove(SE3([piston_RaCx, piston_RaCy 0]))
%     piston_Rb.globalMove(SE3([piston_RbCx, piston_RbCy 0]))

    piston_La.globalMove(SE3([piston_LaCx, piston_LaCy, 0]))
    piston_Lb.globalMove(SE3([piston_LbCx, piston_LbCy, 0]))
    piston_Ra.globalMove(SE3([piston_RaCx, piston_RaCy 0]))
    piston_Rb.globalMove(SE3([piston_RbCx, piston_RbCy 0]))
%
%     pulley_aObj.globalMove(SE3([pulley_aCx, pulley_aCy, 0]))
%     pulley_bObj.globalMove(SE3([pulley_bCx, pulley_bCy, 0]))

%% plot the pulley
    ph=0:.1:pi;
    for i=1:length(ph)
      pulley_ax(i)=pulley_aCx+pulley_aR*cos(ph(i));
      pulley_ay(i)=pulley_aCy+pulley_aR*sin(ph(i));

    end

    fill(pulley_ax,pulley_ay,[1,1,.5])
    %% inner piston that moves
    pistonInner_La.globalMove(SE3([pistonInner_LaCx, pistonInner_LaCy, 0]))
    pistonInner_Lb.globalMove(SE3([pistonInner_LbCx, pistonInner_LbCy, 0]))
    pistonInner_Ra.globalMove(SE3([pistonInner_RaCx, pistonInner_RaCy, 0]))
    pistonInner_Rb.globalMove(SE3([pistonInner_RbCx, pistonInner_RbCy, 0]))

    %% ploting the belt from pulley to the inner piston
    plt(1)=plot([pulley_aCx+pulley_aR*cos(-pi) pistonInner_LaCx],[pulley_aCy+pulley_aR*sin(-pi) pistonInner_LaCy-pistonIn_hL*.5],'r','LineWidth',4);
    plt(2)=plot([pulley_aCx+pulley_aR*cos(0) pistonInner_LbCx],[pulley_aCy+pulley_aR*sin(0) pistonInner_LbCy-pistonIn_hL*.5],'r','LineWidth',4);
    plt(3)=plot([pulley_bCx+pulley_bR*cos(-pi) pistonInner_RaCx],[pulley_bCy+pulley_bR*sin(-pi) pistonInner_RaCy-pistonIn_hR*.5],'r','LineWidth',4);
    plt(4)=plot([pulley_bCx+pulley_bR*cos(0) pistonInner_RbCx],[pulley_bCy+pulley_bR*sin(0) pistonInner_RbCy-pistonIn_hR*.5],'r','LineWidth',4);

    %% ploting the diaphragm
    ph=0:.1:pi;
    for i=1:length(ph)
        diaALx(i)=piston_LaCx+piston_wL*.5*cos(ph(i));
        diaALy(i)=(piston_LaCy+(piston_hL*.5)-diaphragm_LBaseH)+(pistonInner_LaCy+(pistonIn_hL*.5)-(piston_LaCy+(piston_hL*.5)-diaphragm_LBaseH))*sin(ph(i));
        diaBLx(i)=piston_LbCx+piston_wL*.5*cos(ph(i));
        diaBLy(i)=(piston_LbCy+(piston_hL*.5)-diaphragm_LBaseH)+(pistonInner_LbCy+(pistonIn_hL*.5)-(piston_LbCy+(piston_hL*.5)-diaphragm_LBaseH))*sin(ph(i));
        diaARx(i)=piston_RaCx+piston_wR*.5*cos(ph(i));
        diaARy(i)=(piston_RaCy+(piston_hR*.5)-diaphragm_RBaseH)+(pistonInner_RaCy+(pistonIn_hR*.5)-(piston_RaCy+(piston_hR*.5)-diaphragm_RBaseH))*sin(ph(i));
%         diaBLx(i)=piston_LbCx+piston_wL*.5*cos(ph(i));
%         diaBLy(i)=(piston_LbCy+(piston_hL*.5)-diaphragm_LBaseH)+(pistonInner_LbCy+(pistonIn_hL*.5)-(piston_LbCy+(piston_hL*.5)-diaphragm_LBaseH))*sin(ph(i));
%         diaARx(i)=piston_RaCx+piston_wR*.5*cos(ph(i));
%         diaARy(i)=(piston_RaCy+(piston_hR*.5)-diaphragm_RBaseH)+(pistonInner_RaCy+(pistonIn_hR*.5)-(piston_RaCy+(piston_hR*.5)-diaphragm_RBaseH))*sin(ph(i));
        diaBRx(i)=piston_RbCx+piston_wR*.5*cos(ph(i));
        diaBRy(i)=(piston_RbCy+(piston_hR*.5)-diaphragm_RBaseH)+(pistonInner_RbCy+(pistonIn_hR*.5)-(piston_RbCy+(piston_hR*.5)-diaphragm_RBaseH))*sin(ph(i));
    end
   plt(5)= fill(diaALx,diaALy,[1,0,1]);
   plt(6)= fill(diaBLx,diaBLy,[1,0,1]);
   plt(7)= fill(diaARx,diaARy,[1,0,1]);

%    plt(6)= fill(diaBLx,diaBLy,[1,0,1]);
%    plt(7)= fill(diaARx,diaARy,[1,0,1]);
   plt(8)= fill(diaBRx,diaBRy,[1,0,1]);
    %%% ploting the water tubes
    water_ALx=piston_LaCx+piston_wL*.5*cos(pi/2);
    water_ALy=(piston_LaCy+(piston_hL*.5))+(pistonInner_LaCy+(pistonIn_hL*.5)-(piston_LaCy+(piston_hL*.5)))*sin(pi/2);
    water_BLx=piston_LbCx+piston_wL*.5*cos(pi/2);
    water_BLy=(piston_LbCy+(piston_hL*.5))+(pistonInner_LbCy+(pistonIn_hL*.5)-(piston_LbCy+(piston_hL*.5)))*sin(pi/2);


    water_ARx=piston_RaCx+piston_wR*.5*cos(pi/2);
    water_ARy=(piston_RaCy+(piston_hR*.5))+(pistonInner_RaCy+(pistonIn_hR*.5)-(piston_RaCy+(piston_hR*.5)))*sin(pi/2);
    water_BRx=piston_RbCx+piston_wR*.5*cos(pi/2);
    water_BRy=(piston_RbCy+(piston_hR*.5))+(pistonInner_RbCy+(pistonIn_hR*.5)-(piston_RbCy+(piston_hR*.5)))*sin(pi/2);



    plt(9)= plot([water_ALx water_ALx],[water_ALy 5],'b','LineWidth',6);
    plt(10)= plot([water_ALx water_BRx],[5 5],'b','LineWidth',6);
    plt(11)= plot([water_BRx water_BRx],[5 water_BRy],'b','LineWidth',6);

    plt(12)= plot([water_BLx water_BLx],[water_BLy 4.5],'b','LineWidth',6);
    plt(13)= plot([water_BLx water_ARx],[4.5 4.5],'b','LineWidth',6);
    plt(14)= plot([water_ARx water_ARx],[4.5 water_ARy],'b','LineWidth',6);
    
    plt(15)=plot([pulley_aCx pulley_aCx+pulley_aR*cos(input)],[pulley_aCy pulley_aCy+pulley_aR*sin(input)],'b','LineWidth',4);
    plt(20)=plot([pulley_bCx pulley_bCx+pulley_bR*cos(output)],[pulley_bCy pulley_bCy+pulley_bR*sin(output)],'b','LineWidth',4);
    %%%% ploting the needle
    
    plt(21)=plot([pulley_bCx+pulley_bR*cos(output) pulley_bCx+pulley_bR*cos(output)+(2.5^2-(pulley_bCy+pulley_bR*sin(output)^2))^(1/2)],[pulley_bCy+pulley_bR*sin(output) 2],'b','LineWidth',4)

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
