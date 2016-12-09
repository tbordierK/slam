classdef Environment2D
   properties
      features
      robot_position
   end
   methods
      function obj = Environment2D(val,robot)
          obj.features = val;
          obj.robot_position = robot;
      end
      
      function obj = moveRobot(robot)
          obj.robot_position = robot;
      end
      
   end
end