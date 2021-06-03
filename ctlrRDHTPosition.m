function F = ctlrRDHTPosition(t,X,p)
    % Find desired linear force on bel
    
    x_2 = X(5,:);
    dx_2 = X(6,:);
    theta_2 = X(7,:);
    dtheta_2 = X(8,:);
    
    if t < 1/(2*p.freq)
        y = p.ampli*-cos(p.freq*2*pi*t);
        dy = p.ampli*p.freq*2*pi*sin(p.freq*2*pi*t);
    else
        y = p.des_theta;
        dy = 0;
    end
   
    % ddy = -amp*(freq*2*pi)^2*cos(freq*2*pi.*t);

F =20*(y-theta_2)+.1*(dy-dtheta_2);
% F = 0;

if F > 20
    F = 20;
elseif F < -20
    F = -20;
end

% F=0;
end