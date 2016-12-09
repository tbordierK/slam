function [H_t_bar,b_t_bar] = SEIF_motion_update(previous_H_t,previous_b_t,previous_mu_t,u_t,g,U_t,A_t)
%This function computes the motion update
%   Inputs:
%       previous_H_t: previous information (at step t-1)
%       previous_b_t: previous information vector (at step t-1)
%       previous_mu_t: knowledge of the mean mu at step t-1
%       u_t: robot controls a step t
%       g: the motion model, this a deterministic function of the previous state and the controls input.
%       U_t: Covariance matrix of the stochastic part of the robot motion model
%       A_t: Compute gradient of g at (previous_mu_t,u_t)

% All the inputs can probably be refactored.

    
    n = size(previous_b_t);
    
    % Dimension of pose vector (x,y)
    k = 2;
    Sx = [eye(k),zeros(k,n)];
    delta_t = g(previous_mu_t,u_t);
    
    psi_t = eye(n) + Sx\(eye(n)+inv(Sx.'*A_t*Sx))*(Sx.');
    H_prime = psi_t.'*previous_H_t*psi_t;
    delta_H = H_prime*Sx\(inv(U_t)+Sx.'*H_prime*Sx)*(Sx.')*H_prime;
    H_t_bar = H_prime - delta_H;
    b_t_bar = previous_b_t - previous_mu_t.'*(delta_H - previous_H_t + H_prime) + delta_t.'*H_t_bar;

end
