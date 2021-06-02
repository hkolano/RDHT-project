function F = ctlrRDHTPosition(t,X,freq)


amp=.5;


y = amp*sin(freq*2*pi.*t);
dy = amp*freq*2*pi*cos(freq*2*pi.*t);
F =100*(y-X(7,:))+.005*(dy-X(8,:));

% F=0;
end