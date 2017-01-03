function [H_t,b_t] = SEIF_update_measurement_2(H_t_bar,b_t_bar,mu_t,z_t,z_t_hat,Z,n,id,robot)
%This function compute the step of measurement update in SEIF
%   Input:
%   H_bar_t: the information matrix predicted in the step motion
%                 update
%   b_bar_t: the information vector predicted in the step motion
%   mu_t: the state predicted in the step motion
%   z_t:             observations for the robot position and the features
%                    observed
%   Z: covariance matrix for the noise in the observations models
%   output:
%   H_t: the information matrix update
%   b_t: the information vector update

    inv_Z = pinv(Z);
    
    C_t = zeros(n,2);
    C_t(1:2,1:2) = -eye(2);
    C_t((2*id+1):(2*id+2),1:2) = eye(2);
   
    H_t = H_t_bar + C_t*inv_Z*C_t';
    b_t = b_t_bar + ((z_t - z_t_hat+C_t'*mu_t)'*inv_Z*C_t')';
    
    
end

