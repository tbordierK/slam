function [mu_t] = SEIF_state_estimation_2(H_t,b_t,mu_prediction_t,K,id,n)
%This function estimate the expectation of state for the step
%sparsifisation
%   Input:
%   H_t: information matrix output of measurement step
%   b_t: information vector output of measurement step
%   mu_prediction_t: the state predicted by step motion update
%   K: number of iteration for the gradient descend
%   output:
%   mu_t: estimated state

    S_i = zeros(n,2);
    S_i((2*id):((2*id+1)),1) = 1;
    S_i((2*id):((2*id+1)),2) = 1;
    
    v = mu_prediction_t;
    
    for k=1:K
        v((2*id+1):((2*id+2)),1)  = pinv(S_i'*H_t*S_i)*S_i'*(b_t - H_t*v + H_t*S_i*(S_i')*v);
    end
    
    mu_t = v;
end

