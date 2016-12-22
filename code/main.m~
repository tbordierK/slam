m=10; %number of features
n=2*m+2; %dimension of the state vector
% Dimension = 2

nb_iterations=10; %number of iterations
K=10; %number of iterations for the computation of the mean estimate

%Initialization
previous_H_t=zeros(n,n);
previous_b_t=zeros(n,1);
previous_mu_t=zeros(n,1); 
u_t=ones(n,1); %constant control input
U_t=eye(2); % Covariance matrix of the stochastic part of the robot motion model
A_t=eye(n); %Compute gradient of g at (previous_mu_t,u_t)

%Iteration
for k=1:nb_iterations
    [H_t_bar,b_t_bar] = SEIF_motion_update(previous_H_t,previous_b_t,previous_mu_t,u_t,g,U_t,A_t,n);
    mu_t = SEIF_state_estimation(H_t_bar,b_t_bar,mu_prediction_t,K,n);
    [H_t,b_t] =SEIF_update_measurement(H_prediction_t,b_prediction_t,mu_prediction_t,z_t,H_t_1,b_t_1 ,Z_inverse,n);
    [H_t_tild,b_t_tild] = SEIF_sparsification(H_t,b_t,mu_t,n);
end
