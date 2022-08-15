function [q] = system_scara(q, T, ts, k)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = dynamic_scara(q, T, k);   % new
k2 = dynamic_scara(q + ts/2*k1, T, k); % new
k3 = dynamic_scara(q + ts/2*k2, T, k); % new
k4 = dynamic_scara(q + ts*k3, T, k); % new
q = q +ts/6*(k1 +2*k2 +2*k3 +k4); % new

end

