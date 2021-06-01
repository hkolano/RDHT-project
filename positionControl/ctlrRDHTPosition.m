function F = ctlrRDHTPosition(t,X,p)
    % Find desired linear force on bel
    
    x_2 = X(5,:);
    dx_2 = X(6,:);
    theta_2 = X(7,:);
    dtheta_2 = X(8,:);
    % ddy = -amp*(freq*2*pi)^2*cos(freq*2*pi.*t);


%  freq=1;

y = p.ampli*sin(p.freq*2*pi.*t);
dy = p.amp*p.freq*2*pi*cos(p.freq*2*pi.*t);
F =210*(y-theta_2)+.05*(dy-dtheta_2);

% F=0;
end