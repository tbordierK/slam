function [X_features_estimated,Y_features_estimated] = slam_map(mu,env_features,robot,ids,X_previous_map,Y_previous_map)

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

X_features_estimated = X_previous_map;
Y_features_estimated = Y_previous_map;

for k=ids
    X_features_estimated(k) = mu(2*k+1)+mu(1);
    Y_features_estimated(k) = mu(2*k+2)+mu(2);
end

scatter(X_features_estimated,Y_features_estimated,100,'o');
hold on
% scatter(X_features_real,Y_features_real,140,'MarkerEdgeColor',[0 .5 .5],...
%               'MarkerFaceColor',[0 .7 .7],...
%               'LineWidth',1.5)
scatter(X_features_real,Y_features_real,140,'+')
hold on
scatter(robot_position_x,robot_position_y,'filled','o')
circle(robot_position_x,robot_position_y,robot.sensor_range);
legend('estimated features','real features','robot position');
axis([(-1) 5 (-2) 6]);
end