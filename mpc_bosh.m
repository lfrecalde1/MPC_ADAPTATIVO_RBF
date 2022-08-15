function [f,solver,args] = mpc_bosh(bounded, N, ts)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;



%% Definicion de las restricciones en las acciones de control
T1_max = bounded(1); 
T1_min = bounded(2);

T2_max = bounded(3);
T2_min = bounded(4);


%% Generacion de las variables simbolicas de los estados del sistema
q_1 = SX.sym('q1'); 
q_2 = SX.sym('q2');
q_1p = SX.sym('q_1p');
q_2p = SX.sym('q_2p');


%% Definicion de cuantos estados en el sistema
states = [q_1;q_2;q_1p;q_2p];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
T_1 = SX.sym('T_1');
T_2 = SX.sym('T_2');


%% Defincion de cuantas acciones del control tiene el sistema
controls = [T_1;T_2]; 
n_control = length(controls);


%% INERTIAL MATRIX
M_1 = [[                22960000/(3*(1314294*cos(q_2) - 75843*cos(q_2)^2 + 13152437)),  -(15000*(53*cos(q_2) + 51))/(1314294*cos(q_2) - 75843*cos(q_2)^2 + 13152437)]
      [ -(15000*(53*cos(q_2) + 51))/(1314294*cos(q_2) - 75843*cos(q_2)^2 + 13152437), (2500*(636*cos(q_2) + 5759))/(1314294*cos(q_2) - 75843*cos(q_2)^2 + 13152437)]];
 

%% Coriolis and Centripetas Forces
C = [31.8192-0.0954*sin(q_2)*q_2p -0.0954*sin(q_2)*(q_1p+q_2p);
    0.3418*sin(q_2)*q_1p 12.5783];

%% Friction Forces
f = [1.0256*sign(q_1p);...
    1.7842*sign(q_2p)];
     
%% Space State Model
A = [zeros(2,2), eye(2,2);...
     zeros(2,2), -M_1*C];
 
B = [zeros(2,2);...
     M_1];

F = [zeros(2,1);...
     -M_1*f];
          
rhs=(A*states+B*controls+F);
f = Function('f',{states,controls},{rhs});

%% Definicion de kas funciones del sistema
U = SX.sym('U',n_control,N);
P = SX.sym('P',(n_states) + N*(n_states+2)+12);

%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion

%%EMPY VECTOR ERRORS
hep = [];
hev = [];
%% EMPY VECTOR CONTROL VALUES
u = [];

%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state

g = [g;X(:,1)-P(1:4)]; % initial condition constraints

% fee = P((n_states)+N*(n_states+2)+1:(n_states)+N*(n_states+2)+2);
W_1 = SX.zeros(10,1);
W_2 = SX.zeros(10,1);
%chi_estimate = P((n_states)+N*(n_states+2)+1:(n_states)+N*(n_states+2)+12);
%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    %% States system
    st = X(:,k); 
    q = X(:,k);
    v = X(3:4,k);

    %% Funcion costo a minimizar 
    hep = [hep;X(1:2,k)-P(4*k+1:4*k+2)];
    hev = [hev;X(3:4,k)-P(4*k+3:4*k+4)];
    

    %% auxiliar variables
    qd =  P(4*k+1:4*k+2);
    qdp = P(4*k+3:4*k+4);
    qdpp = P(4*k+5:4*k+6);

    [f_estimate, W_1, W_2] = neural_network_estimation_casadi(q, qd, qdp, qdpp, W_1, W_2, ts);
    
    con = U(:,k);
    u = [u;con];

    %% Actualizacion del sistema usando Euler runge kutta
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + ts/2*k1, con); % new
    k3 = f(st + ts/2*k2, con); % new
    k4 = f(st + ts*k3, con); % new
    st_next_RK4 = st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 

    
end

% Cost final 
Q = 1*eye(size(hep,1));
Qv = 0.1*eye(size(hev,1));
R = 0.0000001*eye(size(u,1));


% FINAL COST
obj = hep'*Q*hep+u'*R*u + hev'*Qv*hev;


% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,4*(N+1),1);reshape(U,2*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:4*(N+1)) = -1e-20;  %-1e-20  %Equality constraints
args.ubg(1:4*(N+1)) = 1e-20;  %1e-20   %Equality constraints



args.lbx(1:4:4*(N+1),1) = -inf; %state x lower bound
args.ubx(1:4:4*(N+1),1) = inf;  %state x upper bound

args.lbx(2:4:4*(N+1),1) = -inf; %state y lower bound
args.ubx(2:4:4*(N+1),1) = inf;  %state y upper bound

args.lbx(3:4:4*(N+1),1) = -inf; %state z lower bound
args.ubx(3:4:4*(N+1),1) = inf;  %state z upper bound

args.lbx(4:4:4*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:4:4*(N+1),1) = inf;  %state theta upper bound




%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(4*(N+1)+1:2:4*(N+1)+2*N,1) = T1_min;  %
args.ubx(4*(N+1)+1:2:4*(N+1)+2*N,1) = T1_max;  %

args.lbx(4*(N+1)+2:2:4*(N+1)+2*N,1) = T2_min;  %
args.ubx(4*(N+1)+2:2:4*(N+1)+2*N,1) = T2_max;  % 



end