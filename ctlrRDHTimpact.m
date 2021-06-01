function F = ctlrRDHTPosition(t,X,p)
    % Find desired linear force on bel
    
    x_2 = X(5,:);
    dx_2 = X(6,:);
    theta_2 = X(7,:);
    dtheta_2 = X(8,:);
    % ddy = -amp*(freq*2*pi)^2*cos(freq*2*pi.*t);

y = p.obstacle_height;
dy = 0;
F =210*(y-theta_2)+.05*(dy-dtheta_2);

% F=0;
end