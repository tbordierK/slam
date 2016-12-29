function [H_t_tild,b_t_tild] = SEIF_sparsification(H_t,b_t,mu_t,n)
% Input:
% H_t: information matrix computed at the measurement update state
%b_t: information vector computed at the measurement update state
%mu_t: the state predicted at the measurement update state

%Output:
%H_t_tild: sparsed information matrix
%b_t_tild: sparsed information vector

k=2;
n = size(b_t,1);

% Define the set Y_plus, Y_zero, Y_min
% Y_plus = {active features s.t. the coeff in H_t is below threshold} ? for example, I think we
% have to define the criteria for sparsification ourselves. 
% Y_zero = {active features s.t. the coeff in H_t is above threshold}
% Y_min = {passive features}

threshold = 2;
Y_plus = find(H_t(1,:)<threshold & H_t(1,:)>0);
Y_0 = find(H_t(1,:)>threshold &  H_t(1,:)~=0);
Y_plus
Y_0

% Projection matrices

S_x = [eye(2),zeros(2,2*n)];  

S_Y0 = zeros(2,n);
S_Y0(1:2,(1+2*Y_0):end) = eye(2);

S_x_Y0 = [eye(2),zeros(2,2*n)];   
S_x_Y0(1:2,(1+2*Y_0):end) = eye(2);

S_x_Y0_Yplus = [eye(2),zeros(2,2*n)];   
S_x_Y0_Yplus(1:2,(1+2*Y_0):end) = eye(2);
for j=Y_plus
    S_x_Y0_Yplus(1:2,(1+2*j):end) = eye(2);
end

% Pre-computations

H_t_prim = S_x_Y0_Yplus*S_x_Y0_Yplus'*H_t*S_x_Y0_Yplus*S_x_Y0_Yplus';
H_t_1 = H_t_prim - H_t_prim*S_Y0*inv(S_Y0'*H_t_prim*S_Y0)*S_Y0'*H_t_prim;
H_t_2 = H_t_prim - H_t_prim*S_x_Y0*inv(S_x_Y0'*H_t_prim*S_x_Y0)*S_x_Y0'*H_t_prim;
H_t_3 = H_t - H_t*S_x*inv(S_x'*H_t*S_x)*S_x'*H_t;

% We compute H_t_tild and b_t_tild

H_t_tild = H_t_1 - H_t_2 + H_t_3;
b_t_tild = b_t + mu_t'*(H_t_tild - H_t);


end