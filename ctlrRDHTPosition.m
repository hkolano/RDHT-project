function F = ctlrRDHTPosition(t,X,p)
    % Find desired linear force on bel
    
    x_2 = X(5,:);
    dx_2 = X(6,:);
    theta_2 = X(7,:);
    dtheta_2 = X(8,:);
    
    if t < 1/(2*p.freq)
        y = p.ampli*-cos(p.freq*2*pi*t);
    else
        y = p.des_theta;
    end
   
    % ddy = -amp*(freq*2*pi)^2*cos(freq*2*pi.*t);

dy = 0;
F =10*(y-theta_2)+0*(dy-dtheta_2);

% F=0;
end