function [T] = nominal_control_law(q, qd, qdp, qdpp)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%% Generation reference signal
[vr, vrp, sigma] = auxiliar_reference(q, qd, qdp, qdpp);
T = control_law(q, vr, vrp, sigma);



end

