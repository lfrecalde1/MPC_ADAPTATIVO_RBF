function [qp] = dynamic_scara(q, T, k)
%UNTITLED2 Summary of this function goes here
%   Expantion of the vectors
q1 = q(1);
q2 = q(2);
qp1 = q(3);
qp2 = q(4);

% end
noise = 1;
M_1 = [[                22960000/(3*(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437)),  -(15000*(53*cos(q2) + 51))/(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437)]
      [ -(15000*(53*cos(q2) + 51))/(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437), (2500*(636*cos(q2) + 5759))/(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437)]];
 

%% Coriolis and Centripetas Forces
C = noise*[31.8192-0.0954*sin(q2)*qp2 -0.0954*sin(q2)*(qp1+qp2);
    0.3418*sin(q2)*qp1 12.5783];

%% Friction Forces
f1 = noise*[1.0256*sign(qp1);...
    1.0842*sign(qp2)];

%% External torques

f2 = 4*[1.0256*sin(2*qp1);...
    1.0842*cos(2*qp2)];

%% Space State Model
A = [zeros(2,2), eye(2,2);...
     zeros(2,2), -M_1*C];
 
B = [zeros(2,2);...
     M_1];

F1 = [zeros(2,1);...
     -M_1*f1];
 
F2 = [zeros(2,1);...
     -eye(2)*f2];
 
qp = A*q+B*T+F1+F2;
end

