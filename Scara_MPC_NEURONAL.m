%%------------------------------- Control Adaptativo --------------------%%
%% Clean Variable
clc, clear all, close all;

%% Time Definition
t_s = 0.01;
to =0;
t_final = 40;
t = (to:t_s:t_final);

%% System initial conditions
q = zeros(4,length(t)+1);

q(:,1) = [0.09;-0.09;0;0];
H = q(:,1);
%% Control Variable
% T = zeros(2,length(t));

%% Desired values of the system 
L1=0.445; % articulacin 1
L2=0.355; % articulacion 2

hx(1)=L1*cos(q(1,1))+L2*cos(q(1,1)+q(2,1));
hy(1)=L1*sin(q(1,1))+L2*sin(q(1,1)+q(2,1));

%Condicones Deseadas

% c) Condiciones deseada - TAREA DESEADA
qd1 = sin(1.8*t);  
qd2 = cos(1.8*t);
qd = [qd1;qd2];

% Derivada de la senal de referencia
qd1p = 1.8*cos(1.8*t);  
qd2p = -1.8*sin(1.8*t);
qdp = [qd1p;qd2p];


qd1pp = -1.8*1.8*sin(1.8*t);
qd2pp = -1.8*1.8*cos(1.8*t);
qdpp = [qd1pp;qd2pp];

%% General vector of desired angles and angular velocities
hd = [qd;qdp;qdpp];

%% Proyectiton to plot the position od the robot
hxd=L1*cos(qd1)+L2*cos(qd1+qd2); % Posicion deseada (no es parte del control solo visualizacion)
hyd=L1*sin(qd1)+L2*sin(qd1+qd2);

global neurons
neurons = 10;
%% INital values neural network
W_1 = 0.0*ones(neurons,1);
W_2 = 0.0*ones(neurons,1);

%% General vector of desired angles and angular velocities
hd = [qd;qdp;qdpp];

%% Proyectiton to plot the position od the robot
hxd=L1*cos(qd1)+L2*cos(qd1+qd2); % Posicion deseada (no es parte del control solo visualizacion)
hyd=L1*sin(qd1)+L2*sin(qd1+qd2);


%% Definicion del horizonte de prediccion
N = 10; 

%% Definicion de los limites de las acciondes de control
bounded = [70; -70; 70; -70];
%% Definicion del vectro de control inicial del sistema
vc = zeros(N,2);
H0 = repmat(H,1,N+1)'; 

%% OPTIMIZATION SOLVER
[f, solver, args] = mpc_bosh(bounded, N, t_s);
for k = 1:length(t)-N
    
    %% Control Law
    q1e(k) =  qd1(k)-q(1,k) ;
    q2e(k) =  qd2(k)-q(2,k);
    
    
    q1pe(k) = qd1p(k)-q(3,k);
    q2pe(k) = qd2p(k)-q(4,k);
    
    
    %% General control vectors error
    qe = [q1e(k) q2e(k)]';
    qpe = [q1pe(k) q2pe(k)]';
    
    [f_estimate(:,k), W_1(:,k+1), W_2(:,k+1)] = neural_network_estimation(q(:,k), qd(:,k), qdp(:,k), qdpp(:,k), W_1(:,k), W_2(:,k), t_s);
    norm_estimate(k) = norm(f_estimate(:,k));
    
    %% Real function system
    [f_real(:,k)] =  real_friction(q(:,k));
    norm_real(k) = norm(f_real(:,k));
    
    tic
    %% control Law MPC
    [H0, control] = NMPC_bosh(q(:,k), hd, k, H0, vc, args, solver ,N);
    
    control(1,:) = control(1,:)'+ f_estimate(:,k);
    
%     control(1,:) = control(1,:)';
 
    T(:,k) = control(1,:)';
    toc
    
    %% System evolution
    q(:,k+1) =   system_scara(q(:,k), T(:,k), t_s, k);
    
    %% Geometry system
    hx(k+1)=L1*cos(q(1,k+1))+L2*cos(q(1,k+1)+q(2,k+1)); % no es parte del controlador Graficar
    hy(k+1)=L1*sin(q(1,k+1))+L2*sin(q(1,k+1)+q(2,k+1));
    
    %% New values MPC
    vc = [control(2:end,:);control(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
    
end
paso = 1;

H1 = bosch(q(1,1),q(2,1)); hold on
H2 = plot(hx(1),hy(1),'g','LineWidth',2); hold on
H3 = plot(hxd,hyd,'*r');
axis equal;

for i=1:100:length(t)-N
delete(H1)
delete(H2)
%drawnow 
H1 = bosch(q(1,i),q(2,i)); hold on
H2 = plot(hx(1:i),hy(1:i),'g','LineWidth',2);
disp("i: "+i);
pause(0)
end



figure(3)
subplot(2,1,1)
    plot(t(1:length(q1e)),q1e,'g','LineWidth',2); hold on
    plot(t(1:length(q1e)),q2e,'m','LineWidth',2); grid
    legend('Error de q1','Error de q2')
    title('Errores de Control')
    %xlabel('Tiempo [s]'); ylabel('[rad]');
subplot(2,1,2)
    plot(t(1:length(q1e)),q1pe,'g','LineWidth',2); hold on
    plot(t(1:length(q1e)),q2pe,'m','LineWidth',2); grid
    legend('Velocidad de Error de q1p','Velocidad de Error de q1p')
    xlabel('Tiempo [s]'); ylabel('[rad/s]');
    
figure(4)

    plot(t(1:length(q1e)),T(1,:),'g','LineWidth',2); hold on
    plot(t(1:length(q1e)),T(2,:),'m','LineWidth',2); grid
    legend('T1','T2')
    title('Valores de control')
    xlabel('Tiempo [s]'); ylabel('[Nm]');
    

figure(8)

plot(t(1:length(f_estimate)),f_estimate(1,:),'--','LineWidth',2); hold on
plot(t(1:length(f_real)),f_real(1,:),'-','LineWidth',2); hold on

legend('F estimate 1','F real 1')
title('Torque Estimation')
xlabel('Tiempo [s]'); ylabel('None');

figure(9)

plot(t(1:length(f_estimate)),f_estimate(2,:),'--','LineWidth',2); hold on
plot(t(1:length(f_real)),f_real(2,:),'-','LineWidth',2); hold on

legend('F estimate 2','F real 2')
title('Torque Estimation')
xlabel('Tiempo [s]'); ylabel('None');


%% Generacion del vector general de los errores del sistema
E_mpc_neuronal = [q1e;...
         q2e;...
         q1pe;...
         q2pe];
     
save("errores_mpc_neuronal.mat","E_mpc_neuronal");