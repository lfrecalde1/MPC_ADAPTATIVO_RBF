function [f_estimate, W_1, W_2] = neural_network_estimation(q, qd, qdp, qdpp, W_1, W_2, t_s)


    global neurons;
    %% Matrix values for RBF
    C_1 = linspace(-2.5,2.5,neurons);
    C_2 = linspace(-2.5,2.5,neurons);
    
    %% Internal values RBF
    C_j = [C_1;C_2];
    b_j = 10*ones(neurons,1);
    
    
    h_1 = zeros(neurons,1);
    h_2 = zeros(neurons,1);

    
    %% Control Law
    q1e =  qd(1)-q(1);
    q2e =  qd(2)-q(2);

    q1pe = qdp(1)-q(3);
    q2pe = qdp(2)-q(4);

    %% Vector neural network
%     ze1 = [q1e; q1pe];
%     ze2 = [q2e; q2pe];
    
    ze1 = [q(1); q(3)];
    ze2 = [q(2); q(4)];
    
    %% Auziliar reference values
    [vr, vrp, sigma] = auxiliar_reference(q, qd, qdp, qdpp);
    
    for j=1:1:length(C_j)
        h_1(j)=exp(-norm(ze1-C_j(:,j))^2/(2*b_j(j)*b_j(j))); %Hidden Layer
        h_2(j)=exp(-norm(ze2-C_j(:,j))^2/(2*b_j(j)*b_j(j))); %Hidden Layer
    end
    
                   
    %% real system value
    [f_real] = f_real_system(q, vr, vrp);
    %% Control law               
%     T = f_real+20*sigma+f_estimate;    
    
    
    %% Adaptation control law
    W1p= 150*h_1*sigma(1);
    W2p= 150*h_2*sigma(2);
                   
    W_1 = W_1 + W1p*t_s;
    W_2 = W_2 + W2p*t_s;
    
    f_estimate = [W_1'*h_1;...
                       W_2'*h_2];
end

