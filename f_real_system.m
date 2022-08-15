function [f_real] = f_real_system(q, vr, vrp)
%UNTITLED4 Summary of this function goes here
q1 = q(1);
q2 = q(2);
qp1 = q(3);
qp2 = q(4);

%% Intertial Matrix
M = [1.7277+0.1908*cos(q2) 0.0918+0.0954*cos(q2);...
    0.0918+0.0954*cos(q2) 0.9184];

%% Coriolis and Centripetas Forces
C = [31.8192-0.0954*sin(q2)*qp2 -0.0954*sin(q2)*(qp1+qp2);
    0.3418*sin(q2)*qp1 12.5783];

%% Friction Forces
f1 = [1.0256*sign(qp1);...
    1.0842*sign(qp2)];

KD = 10*eye(2);
f_real = M*vrp + C*vr + f1;

end

