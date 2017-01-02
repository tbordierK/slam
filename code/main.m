clf;

m=3; %number of features
n=2*m+2; %dimension of the state vector


nb_iterations = 10; %number of iterations
K = 100 ; %number of iterations for the computation of the mean estimate

 
sensor_range = 2;
robot = Robot2D([0.2,0.2,sensor_range]); 
env_features = [1,1,2,3,1,0];

%Initialization
previous_H_t=rand(n,n);
previous_b_t=zeros(n,1);
previous_mu_t=[robot.x_position;robot.y_position;zeros(n-2,1)]; 

u_t=zeros(2,1); %constant control input
U_t=0.0001*eye(2); % Covariance matrix of the stochastic part of the robot motion model
A_t=eye(n); % The  Special case linear dynamics, A_t = 0 always. 

% Define noises
mu_measurement = 0;
Z = 0.0001;


%Iteration

for k = 1:nb_iterations

    display(k)
    [H_t_bar,b_t_bar,mu_t_bar] = SEIF_motion_update(previous_H_t,previous_b_t,previous_mu_t,u_t,A_t,U_t,n);

    % Returns a list of features within robot range and their distance
    [z_t_hat,ids] = robot.measure(env_features);
    nb_features = size(ids,2);

    if nb_features~=0  %measure 

        % If the robot measures a feature
        for i=1:nb_features

            id = ids(i);
            mu_t = SEIF_state_estimation_2(H_t_bar,b_t_bar,mu_t_bar,K,id,n);

            % The noise for measurement is included here. 
            eps_t = Z*randn(2,1)+mu_measurement;
            z_t = z_t_hat(1:2,i) + eps_t; 

            [H_t,b_t] = SEIF_update_measurement_2(H_t_bar,b_t_bar,mu_t,z_t(1:2),z_t_hat(1:2,i),Z,n,id);
            [H_t_tild, b_t_tild] = SEIF_sparsification(H_t,b_t,mu_t,n);
        end 

       

    else   %no measure

        

        % If the robot does not measure a feature, what do we do?

        H_t_tild = H_t_bar;
        b_t_tild = b_t_bar;
        mu_t = mu_t_bar;

    end

    

    %update robot position

    robot.x_position = mu_t(1); 
    robot.y_position = mu_t(2);

    previous_H_t = H_t_tild;
    previous_b_t = b_t_tild;
    previous_mu_t = mu_t;
    
    mu_t

    
    % Plots the map (estimated robot position, estimated features, real position of features)
    clf;
    slam_map(mu_t,env_features);  
    display(sprintf('Press any key to proceed')); 
    pause;
    

    

end

 