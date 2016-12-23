function y=h(state_t,id_feature)
%Returns the distance between the robot and the observed feature
%Input: 
%state_t
%id_feature: id of the observed feature
robot_position=state_t(1:2,1);
feature_position=state_t((2*id_feature+1):(2*id_feature+2));
y=norm(robot_position-feature_position)^2;
end