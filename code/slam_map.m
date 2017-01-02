function y= slam_map(mu,env_features)

n = size(mu,1);
m = floor(n/2-1); %number of features

X_features_real = zeros(m,1);
Y_features_real = zeros(m,1);

for k=0:(m-1)
    X_features_real(k+1) = env_features(2*k+1);
    Y_features_real(k+1) = env_features(2*k+2);
end

robot_position_x = mu(1);
robot_position_y = mu(2);

X_features_estimated = zeros(m,1);
Y_features_estimated = zeros(m,1);

for k=1:m
    X_features_estimated(k) = mu(2*k+1);
    Y_features_estimated(k) = mu(2*k+2);
end

X = vertcat(X_features_estimated,X_features_real);
Y = vertcat(Y_features_estimated,Y_features_real);

scatter(X_features_estimated,Y_features_estimated,500,'d');
hold on
scatter(X_features_real,Y_features_real,140,'MarkerEdgeColor',[0 .5 .5],...
              'MarkerFaceColor',[0 .7 .7],...
              'LineWidth',1.5)
hold on
scatter(robot_position_x,robot_position_y,'filled','o')
legend('estimated features','real features','robot position');
axis([(-2) 14 (-4) 10]);
end