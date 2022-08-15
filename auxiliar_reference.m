function [vr, vrp, sigma] = auxiliar_reference(q, qd, qdp, qdpp)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
qe = qd-q(1:2);
delta = 1*eye(2);
v = q(3:4);
vd = qdp;
vdp = qdpp;
ve = vd-v;
sigma = ve + delta*qe;
vr = sigma + v;
vrp = vdp + delta*ve;

end