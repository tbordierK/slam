function [H_t_next,b_t_next,mu_t_next] = SEIF_SLAM(H_t,b_t,mu_t)

 % To be defined u,g,U_t,A_t
[H_t_bar,b_t_bar,mu_t_bar] = SEIF_motion_update(H_t,b_t,mu_t,u,g,U_t,A_t);
% To be defined z_t,H_t,b_t,Z_inverse
[H_t,b_t] = SEIF_measurement_update(H_t_bar,b_t_bar,mu_t_bar,z_t,H_t,b_t,Z_inverse);

K = 10;
mu_t = SEIF_state_estimation(H_t,b_t,mu_t_bar,K);

% Waiting for SEIF_sparficiation

end