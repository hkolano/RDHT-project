function [y, dy, ddy] = disturbanceTrajectory(amp, freq, t)
% input: freq = frequency in hz
%        amp = amplitude 
%        t = time (scalar or vector)

y = amp*cos(freq*2*pi.*t);
dy = -amp*freq*2*pi*sin(freq*2*pi.*t);
ddy = -amp*(freq*2*pi)^2*cos(freq*2*pi.*t);

end