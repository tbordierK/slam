function [mu_t] = SEIF_state_estimation(H_t,b_t,mu_prediction_t,K,idxFeature)
%This function estimate the expectation of state for the step
%sparsifisation
%   Input:
%   H_t: information matrix output of measurement step
%   b_t: information vector output of measurement step
%   mu_prediction_t: the state predicted by step motion update
%   K: number of iteration for the gradient descend
%   output:
%   mu_t: estimated state


   
   % index of robot position and the locations of active features
   idxrobotposition = 1;
   idx = [idxrobotposition, idxFeature];
   n_idx = length(idx); % always 2
   H_t_sub = zeros(n_idx,n_idx);
   b_t_sub = zeros(1,n_idx);
   mu_prediction_t_sub= zeros(n_idx,1);
   
   for i=1:n_idx
       for j=1:n_idx
           H_t_sub(i,j) = H_t(idx(i),idx(j));
       end
       b_t_sub(i) = b_t(idx(i));
       mu_prediction_t_sub(i) = mu_prediction_t(i);
   end
   
   mu_t = mu_prediction_t_sub;
   
   v = mu_t;   %%%% not sure about this line ... but v wasn't defined 
   
   for n_iterat =1:K
       tt = zeros(n_idx,1);
       for i = 1:n_idx
           tt(i) = (b_t_sub(i)- H_t_sub(i,:)*v+H_t_sub(i,i)*v(i))/H_t_sub(i,i);
       end
       mu_t = tt;
   end


end

