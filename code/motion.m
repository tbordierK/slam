function y = g_motion(state_t,u_t)  % function g in the article
%the motion model, this a deterministic function of the previous state and the controls input.
y = state_t+u_t;
end

%Thus, for this particular example  we have A_t=Identity