%% Program to control a system using RBF neural networks
clc, clear all, close all;

%% time definition
t_s = 0.05;
t_final = 100;
t = (0:t_s:t_final);

%% Inital states system
x(:,1) = [pi/6;0];


%% Initial control value
u = 0*ones(1,length(t));

%% Desired states system
xd = [sin(0.5*t);0.5*cos(0.5*t);-0.5*0.5*sin(0.5*t)];

kp = 10;
kd = 20;
K = [kp;kd];

%% Paramters of the neural network
%% number neurons
neurons = 10;
%% Matrix values for RBF
C_1 = linspace(-1,1,neurons);
C_2 = linspace(-1,1,neurons);

%% Internal values RBF
C_j = [C_1;C_2];
b_j = 2*ones(neurons,1);

W = 0.1*ones(neurons,1);
h = zeros(neurons,1); 

Q = [50,0;0,50];
F = [0,1;...
     -kp,-kd];
gamma = 50;
A = F';
P = lyap(A,Q);
B = [0;1];
for k = 1:length(t)
    
    %% Vector of erros fo the system 
    e(k) = xd(1,k)-x(1,k);
    ep(k) = xd(2,k)-x(2,k);
    
    %% General vector of erros
    he = [e(k);ep(k)];
    
    for j=1:1:length(C_j)
        h(j)=exp(-norm(he-C_j(:,j))^2/(2*b_j(j)*b_j(j))); %Hidden Layer
    end
    
    %% Neural network output;
    H(:,k) = h;
    
    f_estimate(k) = W(:,k)'*h;
    f_real(k) = -25*x(2,k)+sin(x(1,k));
    
    %% Contro Law
    u(k) = (1/(133))*(-(f_estimate(k))+xd(3,k)+K'*he);
    x(:,k+1) = system_simulation(x(:,k),u(:,k),t_s);
    
    Wp= -gamma*he'*P*B*h;
    W(:,k+1) = W(:,k) + Wp*t_s;
    
end

figure(1);
plot(t,x(:,1:length(t)),'--b');
grid on;
hold on;
plot(t,xd(1:2,1:length(t)),'r');
xlabel('times');ylabel('y');

figure(2);
plot(t,u,'g');
grid on;
xlabel('times');ylabel('u');

figure(3);
plot(t,e,'b');
grid on;
xlabel('times');ylabel('e');


figure(4);
plot(t,f_estimate(:,1:length(t)),'--b');
grid on;
hold on;
plot(t,f_real(1,1:length(t)),'r');
xlabel('times');ylabel('f');

figure(5);
plot(t,W(:,1:length(t)),'r');
hold on;
xlabel('times');ylabel('W');
