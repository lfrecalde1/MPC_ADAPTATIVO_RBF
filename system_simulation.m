function [x] = system_simulation(x,u,ts)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
k1 = system_f(x, u);   % new
k2 = system_f(x + ts/2*k1, u); % new
k3 = system_f(x + ts/2*k2, u); % new
k4 = system_f(x + ts*k3, u); % new
x = x +ts/6*(k1 +2*k2 +2*k3 +k4); % new
end

