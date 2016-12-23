function [H_t,b_t] = SEIF_update_measurement(H_prediction_t,b_prediction_t,mu_prediction_t,H_t_1,b_t_1 ,Z_inverse,n)
%This function compute the step of measurement update in SEIF
%   Input:
%   H_prediction_t: the information matrix predicted in the step motion
%                 update
%   b_prediction_t: the information vector predicted in the step motion
%   mu_prediction_t: the state predicted in the step motion
%   z_t:             observations for the robot position and the features
%                    observed
%   H_t_1:  information matrix for instant t-1
%   b_t_1:   information vector for instant t-1
%   Z_inverse: information matrix for the noise in the observations models
%   output:
%   H_t: the information matrix update
%   b_t: the information vector update

    %Example: 
    idx_robot_position = 1;
    idx_feature = [2,3];

    idx= [idx_robot_position, idx_feature];
    n_idx= length(idx);
    % compute the measurement function h at mu_prediction_t
    h_mu_prediction_t =[] ;
    
    % compute the derivation of measurement function h at mu_prediction_t
    % only compute for feature observed idx_feature and the robot position
    gradient_h_mu_prediction_t = [];
    
    % compute only the component of update
    H_prediction_t_sub = zeros(n_idx,n_idx);
    b_prediction_t_sub = zeros(1,n_idx);
    Z_inverse_sub = zeros(n_idx,n_idx);
    mu_prediction_t_sub = zeros(n_idx,1);
    
    %Computationof z_t, spatial information on the relation of the robot?s pose and the location of a feature
    z_t=h(mu_prediction_t); %don't know how to compute it 
    
    for i = 1:n_idx
        for j=1:n_idx
            H_prediction_t_sub (i,j)=  H_prediction_t(idx(i),idx(j));
            Z_inverse_sub(i,j)=Z_inverse(idx(i),idx(j));
        end
        b_prediction_t_sub (1,i)= b_prediction_t(1,idx(i));
        mu_prediction_t_sub(1,i)=mu_prediction_t(1,idx(i));
    end
    
    %update
    H_t_sub = H_prediction_t_sub + gradient_h_mu_prediction_t*Z_inverse_sub*gradient_h_mu_prediction_t';
    b_t_sub = b_prediction_t_sub + (z_t - h_mu_prediction_t+gradient_h_mu_prediction_t'* mu_prediction_t_sub)'*Z_inverse_sub*gradient_h_mu_prediction_t';
    
    H_t = H_t_1;
    b_t = b_t_1;
    for i = 1:n_idx
        for j=1:n_idx
            H_t(idx(i),idx(j)) = H_t_sub(i,j);
        end
        b_t(idx(i)) = b_t_sub(i);
    end
    
end

