function [xp] = system_f(x,u)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
xp(1,1) = x(2);
xp(2,1) = -25*x(2)+sin(x(1))+133*u;
end

