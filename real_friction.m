function [f2] = real_friction(q)
%UNTITLED2 Summary of this function goes here
%   Expantion of the vectors
q1 = q(1);
q2 = q(2);
qp1 = q(3);
qp2 = q(4);

M_1 = [[                22960000/(3*(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437)),  -(15000*(53*cos(q2) + 51))/(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437)]
      [ -(15000*(53*cos(q2) + 51))/(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437), (2500*(636*cos(q2) + 5759))/(1314294*cos(q2) - 75843*cos(q2)^2 + 13152437)]];
 

f2 = 4*[1.0256*sin(2*qp1);...
    1.0842*cos(2*qp2)];


F2 = [zeros(2,1);...
     -eye(2)*f2];

end

