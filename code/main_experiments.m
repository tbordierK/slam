clf;
K = 10 ; %number of iterations for the computation of the mean estimate
   
sensor_range = 0.5;
robot = Robot2D([1,1,sensor_range]); 
%env_features = [[1,1,2,2,3,3,4,4],[1,1,2,2,3,3,4,4]+[1,0,1,0,1,0,1,0],[1,1,2,2,3,3,4,4]-[1,0,1,0,1,0,1,0]];
env_features = generate_grid(3);
m=size(env_features,2)/2; %number of features
n=2*m+2; %dimension of the state vector

commands = generate_commands();
nb_iterations = size(commands,2)/2;

%Initialization
previous_H_t=eye(n);
for k=1:(m+1)
    previous_H_t(2*k-1,2*k) = 0.5;
    previous_H_t(2*k,2*k-1) = 0.5;
end

previous_mu_t=[robot.x_position,robot.y_position,zeros(1,n-2)]';
mu_t = previous_mu_t;
previous_b_t=previous_H_t*previous_mu_t;

%u_t=+0.1*ones(2,1); %constant control input
U_t=0.0001*eye(2); % Covariance matrix of the stochastic part of the robot motion model
A_t=zeros(n); % The  Special case linear dynamics, A_t = 0 always. 

% Define noises
mu_measurement = 0;
Z = 0.001*eye(2);


%Iteration
ids = [];
X_map = zeros(m,1);
Y_map = zeros(m,1);
slam_map(mu_t,env_features,robot,ids,X_map,Y_map); 
error_1 = [];

for k = 1:nb_iterations
    
    display(k)
     
    % Motion Update
    u_t = commands(:,k);
    [H_t_bar,b_t_bar] = SEIF_motion_update(previous_H_t,previous_b_t,previous_mu_t,u_t,A_t,U_t,n);
    
    % Measure: Robot returns a list of the features around him with
    % relative coordinates.
    [z_t_hat,ids] = robot.measure(env_features);
    nb_features = size(ids,2);
    

    % Testing if at least a feature is within range
    if nb_features~=0   
    
        % If the robot measures a feature
        for i=1:nb_features
            id = ids(i);
              if i~=1
                  [H_t_bar,b_t_bar] = SEIF_motion_update(H_t,b_t,mu_t,zeros(2,1),A_t,U_t,n);
              end
            
            % The noise for measurement is included here. 
            eps_t = Z*randn(2,1)+mu_measurement;
            z_t = z_t_hat(1:2,i) + eps_t; 
             
            % Measurement update
            [H_t,b_t] = SEIF_update_measurement(H_t_bar,b_t_bar,mu_t,z_t(1:2),z_t_hat(1:2,i),Z,n,id);
           
            % State estimation, need to be changed
            mu_t = pinv(H_t)*b_t;
            %mu_t = SEIF_state_estimation_2(H_t,b_t,mu_t,K,id,n);
            
            %Sparsification
            %[H_t, b_t] = SEIF_sparsification(H_t,b_t,mu_t,n);

        end   
    
    else   %no feature measured in the sensor range

        H_t = H_t_bar;
        b_t = b_t_bar;
        display(mu_t);
        mu_t = pinv(H_t)*b_t;

    end

    %update robot position

    robot.x_position = mu_t(1); 
    robot.y_position = mu_t(2);

    previous_H_t = H_t;
    previous_b_t = b_t;
    previous_mu_t = mu_t;
   
    % Plots the map (estimated robot position, estimated features, real position of features)
    %     clf;
 
    [X_map,Y_map] = slam_map(mu_t,env_features,robot,ids,X_map,Y_map);    
    
    %estimation = reshape(mu_t(3:end),2,[])+[mu_t(1),mu_t(2)]';
    %estimation = reshape(estimation,1,[]);
    %error_1(k) = norm(estimation-env_features');
  
end
[X_map,Y_map] = slam_map(mu_t,env_features,robot,ids,X_map,Y_map); 
% figure
% imagesc(H_t_bar);        % draw image and scale colormap to values range
% colorbar;