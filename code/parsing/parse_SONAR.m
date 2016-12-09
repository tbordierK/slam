function [range_readings,number_readings,position,odom,timestamp] = parse_SONAR(line)
    
    split_line = split(line);
    
    % Check that it is a reading and not a command
    if not (split_line(1) == 'SONAR')
        range_readings = -1 ;
        number_readings = -1;
        position = -1;
        odom = -1;
        timestamp = -1;
        return
    end
    
    number_readings = str2num(char(split_line(2)));
    
    % Retrieves the range of the nearby features
    range_readings = zeros(1,number_readings);
    for k=1:number_readings
        range_readings(k) = str2double(char(split_line(k+2)));
    end
    
    % Retrieves x,y theta
    position = zeros(1,3);
    for k=1:3
        position(k) = str2double(char(split_line(k+2+number_readings)));
    end
  
    % Retrieves odom_x odom_y odom_theta
    odom = zeros(1,3);
    for k=1:3
        odom(k) = str2double(char(split_line(3+k+2+number_readings)));
    end
    
    % Timestamp
    timestamp = str2double(char(split_line(2+number_readings+3+3+1)));
    
end