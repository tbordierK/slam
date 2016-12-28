m=3; %number of features
n=2*m+2; %dimension of the state vector
% Dimension = 2

nb_iterations = 10; %number of iterations
K = 10; %number of iterations for the computation of the mean estimate

%Initialization
previous_H_t=rand(n,n);
previous_b_t=rand(n,1);
previous_mu_t=rand(n,1); 
u_t=ones(2,1); %constant control input
U_t=eye(2); % Covariance matrix of the stochastic part of the robot motion model
A_t=eye(n); % Special case linear dynamics, A_t = 0 always. 

sensor_range = 2;
robot = Robot2D([0.5,0.5,sensor_range]);
env_features = [1,1,2,3,1,0];

% Define noises
mu_measurement = zeros(2,1);
Z = 0.01*eye(2);


%Iteration
for k = 1:nb_iterations
    [H_t_bar,b_t_bar,mu_t_bar] = SEIF_motion_update(previous_H_t,previous_b_t,previous_mu_t,u_t,A_t,U_t,n);
    
    [z_t,id] = robot.measure_closest(env_features); % contains h and noise.
    mu_t = SEIF_state_estimation_2(H_t_bar,b_t_bar,mu_t_bar,K,id,n);
    
    % TOPUTINFUNCTIONMODAFUCKA
    R = chol(Z);
    eps_t = repmat(mu_measurement,1,1)+(randn(1,2)*R)';
    
    z_t = z_t + eps_t; 
    [H_t,b_t]=SEIF_update_measurement_2(H_t_bar,b_t_bar,mu_t,z_t,Z,n,id);
    [H_t_tild,b_t_tild] = SEIF_sparsification(H_t,b_t,mu_t,n);
    previous_H_t = H_t_tild;
    previous_b_t = b_t_tild;
    previous_mu_t = mu_t;
end