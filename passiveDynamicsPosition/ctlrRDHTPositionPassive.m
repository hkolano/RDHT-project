function F = ctlrRDHTPositionPassive(t,X,freq)
    % Find desired linear force on bel
    
    x_2 = X(5,:);
    dx_2 = X(6,:);
    theta_2 = X(7,:);
    dtheta_2 = X(8,:);
    % ddy = -amp*(freq*2*pi)^2*cos(freq*2*pi.*t);

amp=.5;
%   freq=1;

y = amp*sin(freq*2*pi.*t);
dy = amp*freq*2*pi*cos(freq*2*pi.*t);
F =2*(y-theta_2)+.01*(dy-dtheta_2);

% F=0;
end