classdef Robot2D
   properties
      x_position
      y_position
      sensor_range
  
   end
   methods
      function obj = Robot2D(val)
         obj.x_position = val(1);
         obj.y_position = val(2);
         obj.sensor_range = val(3);
        
      end
      
      function [surrounding] = measure(obj,env_features)
          % Measures the surrounding features in a radius sensor_range
          % Are we supposed to add some noise here? 
          
          n_features = size(env_features);
          n_features = n_features(2);
          pos = [obj.x_position;obj.y_position];
          
          surrounding = zeros(1,n_features);
          
          for k=1:n_features
              distance = norm(pos-env_features(:,k));
              if distance < obj.sensor_range
                  surrounding(k) = 1;
              end
          end         
      end
      
      
      function [obj,environment] = move(obj,commands,environment)
          % Moves the robot
          %Inputs
          % commands: pair of floats
          % environment object
          % Are we to add noise on the commands? 
          obj.x_position = commands(1); % + ex_t?
          obj.y_position = commands(2); % + ey_t?
          % This moves the robot in the environment
          environment = environment.move_robot(commands);
      end
      
      function [x,y] = getPosition(obj)
          % Returns current position
          x = obj.x_position;
          y = obj.y_position;
         
      end
      
      
   end
end