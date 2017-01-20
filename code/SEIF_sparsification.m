function [H_t_tild,b_t_tild] = SEIF_sparsification(H_t,b_t,mu_t,n)
% Input:
% H_t: information matrix computed at the measurement update state
%b_t: information vector computed at the measurement update state
%mu_t: the state predicted at the measurement update state

%Output:
%H_t_tild: sparsed information matrix
%b_t_tild: sparsed information vector

% Define the set Y_plus, Y_zero, Y_min
% Y_plus = {active features s.t. the pair of coeff in H_t is below threshold} ? for example, I think we
% have to define the criteria for sparsification ourselves. 
% Y_zero = {active features s.t. the pair of coeff in H_t is above threshold}
% Y_min = {passive features}

bound = 10;%maximum number of active features

m = n/2-1; %number of features
robot_links = zeros(1,m);

% We need to define the strength of links based on the values in H_t, but
% given our representation of the states
% (x_robot,y_robot,x_feat1,y_feat2,...) I think we need to find a way to
% define the strenght of the relationship between robot and feature based
% on the the x,y values in the H_t matrix???

for k=1:m
    robot_links(k) = norm([H_t(1,2*k+1),H_t(1,2*k+2)]);
end

b = min(bound,nb_null(robot_links));
[Y_0,Y_plus]=find_active(b,robot_links);

if isempty(Y_0)
    % No sparsification
    H_t_tild = H_t;
    b_t_tild = b_t;
else
    
    % Projection matrices

    S_x = [eye(2),zeros(2,2*m)];  

    S_Y0 = zeros(2,n);
    for j=Y_0
        S_Y0(1:2,(1+2*j):(2+2*j)) = eye(2);
    end
    
    S_x_Y0 = [eye(2),zeros(2,2*m)];
    for j=Y_0
        S_x_Y0(1:2,(1+2*j):(2+2*j)) = eye(2);
    end
 
    S_x_Y0_Yplus = [eye(2),zeros(2,2*m)];
  
  
    for j=Y_plus
        S_x_Y0_Yplus(1:2,(1+2*j):(2+2*j)) = eye(2);
    end
    for j=Y_0
        S_x_Y0_Yplus(1:2,(1+2*j):(2+2*j)) = eye(2);
    end
  
    % Pre-computations
    S_x = S_x';
    S_Y0 = S_Y0';
    S_x_Y0 = S_x_Y0';
    S_x_Y0_Yplus = S_x_Y0_Yplus';

    H_t_prim = S_x_Y0_Yplus*S_x_Y0_Yplus'*H_t*S_x_Y0_Yplus*S_x_Y0_Yplus';
    H_t_1 = H_t_prim - H_t_prim*S_Y0*inv(S_Y0'*H_t_prim*S_Y0)*S_Y0'*H_t_prim;
    H_t_2 = H_t_prim - H_t_prim*S_x_Y0*inv(S_x_Y0'*H_t_prim*S_x_Y0)*S_x_Y0'*H_t_prim;
    H_t_3 = H_t - H_t*S_x*inv(S_x'*H_t*S_x)*S_x'*H_t;
 
    % We compute H_t_tild and b_t_tild

    H_t_tild = H_t_1 - H_t_2 + H_t_3;
    b_t_tild = b_t + (H_t_tild - H_t)*mu_t;
end
end