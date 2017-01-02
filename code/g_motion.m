function y = g_motion(mu_t,u_t)  % function g in the article
%the motion model, this a deterministic function of the previous state and the controls input.
n = size(mu_t,1);
y = zeros(n,1);
y(1:2,1) = u_t;

end
