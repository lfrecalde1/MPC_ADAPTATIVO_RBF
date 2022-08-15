%% Program dor system estimation using RBF neural networks.
%% Clear variables
clc, clear all, close all;

%% time definition
t_s = 0.01;
t_final = 10;
t = (0:t_s:t_final);

y(1) = 0;
u(1) = 0;

%% Memory values
y_1 = 0;
u_1 = 0;


%% number neurons
neurons = 5;
%% Matrix values for RBF
C_1 = linspace(-1,1,neurons);
C_2 = linspace(-1,1,neurons);

%% Internal values RBF
C_j = [C_1:C_2];
b_j = 3*ones(neurons,1);


%% Inital weigths
W = 0.1*ones(neurons,1);
h = zeros(neurons,1); 

%% Learnign rate
n = 0.8;

%% System model
for k = 1:length(t)
    
    %% Control signal
    u(k)= sin(t(k));
    y(k)=u_1^3+y_1/(1+y_1^2);
    x(:,k) = [u(k);y(k)];
    
    %% Neural network proyection
    for j=1:1:length(C_j)
        h(j)=exp(-norm(x(:,k)-C_j(:,j))^2/(2*b_j(j)*b_j(j))); %Hidden Layer
    end
    
    %% Neural network output;
    H(:,k) = h;
    y_estimate(:,k) = W(:,k)'*h;
    
    %% Error vector;
    error(k) = y(:,k)-y_estimate(:,k);
    
    %% delta change
    
    dh = n*error(k)*h;
    
    %% Adaptation law
    W(:,k+1) = W(:,k) + dh;
    %% System simualtion
    
    
    %% Last control value
    y_1 = y(k);
    u_1 = u(k);
end

figure(1);
plot(t,y,'r');
grid on;
xlabel('times');ylabel('y');

figure(2);
plot(t,u,'g');
grid on;
xlabel('times');ylabel('u');

% figure(3);
% plot(x,H,'b');
% grid on;
% xlabel('estado');ylabel('gauss');


figure(4);
plot(t,y,'r');
hold on;
plot(t,y_estimate,'--b');
grid on;
xlabel('times');ylabel('y');

figure(5);
plot(t,W(:,1:length(t)),'r');
hold on;
xlabel('times');ylabel('W');