function [man]=bosch(q1,q2)

% MANIPULADOR
% Dibuja las articulaciones del manipulador bosch, centrado en (0,0)
L1=0.445;
L2=0.355;
%****BRAZO 1*****
x1 = [0 0];
x2 = [L1*cos(q1) L1*sin(q1)];
%****BRAZO 2*****
x3 = [L1*cos(q1)+L2*cos(q1+q2) L1*sin(q1)+L2*sin(q1+q2)];
% ***** Dibuja los brazoz
H01 = line([x1(1) x2(1)],[x1(2) x2(2)]);
set(H01,'Color','r','LineWidth',6);
H02 = line([x2(1) x3(1)],[x2(2) x3(2)]);
set(H02,'Color','k','LineWidth',6);

axis equal;
man = [H01 H02];