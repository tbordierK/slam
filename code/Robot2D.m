classdef Robot2D
   properties
      x_position
      y_position
      sensor_range
      sensor_angle
  
   end
   methods
       
      function obj = Robot2D(val)
          % Initialization of Robot2D
          % input: - val is a 3-array
         obj.x_position = val(1);
         obj.y_position = val(2);
         obj.sensor_range = val(3);
    
        
      end
      
      function [min_distance,id] = measure_closest(obj,env_features)
          % Measures the surrounding features in a radius sensor_range
          % TODO: check is at same position the feature has been measures
          
          n_features = size(env_features,2)/2;
          env_features = reshape(env_features,2,n_features);
          pos = [obj.x_position;obj.y_position];
          
          min_distance = 1000;
          relative_measure = zeros(2,1);
          
          for k=1:n_features
              distance = norm(pos-env_features(:,k));
              if distance < min_distance
                  min_distance = distance;
                  id = k;
                  
                  relative_measure(1) = env_features(1,k) - obj.x_position;
                  relative_measure(2) = env_features(2,k) - obj.y_position;
              end
          end         
      end
      
      function [surrounding] = measure(obj,env_features)
          % Measures the surrounding features in a radius sensor_range
          % TODO: check is at same position the feature has been measures
          
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
      
      
      function [obj] = move(obj,commands)
          % Moves the robot
          %Inputs
          % commands: pair of floats (noisy values)

          obj.x_position = obj.x_position + commands(1); % + ex_t?
          obj.y_position = obj.x_position + commands(2); % + ey_t?
      end
      
      function [x,y] = getPosition(obj)
          % Returns current position
          x = obj.x_position;
          y = obj.y_position;
         
      end
      
      
   end
end