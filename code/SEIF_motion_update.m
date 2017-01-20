function [H_t_bar,b_t_bar] = SEIF_motion_update_2(previous_H_t,previous_b_t,previous_mu_t,u_t,A_t,U_t,n)
%This function computes the motion update
%   Inputs:
%       previous_H_t: previous information (at step t-1)
%       previous_b_t: previous information vector (at step t-1)
%       previous_mu_t: knowledge of the mean mu at step t-1
%       u_t: robot controls a step t
%       U_t: Covariance matrix of the stochastic part of the robot motion model
%       A_t: gradient of g at (previous_mu_t,u_t)

    %Pre-computations
    Sx = [eye(2),zeros(2,n-2)]';
    
    noise = zeros(n,1);
    noise(1:2,1) = U_t*randn(2,1);
    delta_t = g_motion(previous_mu_t,u_t) + noise;
   
    L_t = Sx*inv(inv(U_t)+Sx'*previous_H_t*Sx)*Sx'*previous_H_t;
    H_t_bar = previous_H_t - previous_H_t*L_t;
    
    previous_b_t = previous_b_t';
    b_t_bar = previous_b_t + g_motion(previous_mu_t,u_t)'*previous_H_t-previous_b_t*L_t + g_motion(previous_mu_t,u_t)'*previous_H_t*L_t;
    b_t_bar = b_t_bar';
  
 
   
end