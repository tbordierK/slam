function [position, tv, rv, accel, timestamp] = parse_ODOM(line)
    
    split_line = split(line);
    iter = 1;
    
    % Check that it is a reading and not a command
    if not (split_line(1) == 'ODOM')
        position = 1;
        tv = 1;
        rv = 1;
        accel = 1;
        timestamp = 1;
        return
    end
    
    
    
    % Retrieves the range of the nearby features
    position = zeros(1,3);
    for k=1:3
        position(k) = str2double(char(split_line(k+1)));
        iter = iter + 1;
    end
    
     % tv
    iter = iter+1;
    tv = str2double(char(split_line(iter)));
     % rv
    iter = iter+1;
    rv = str2double(char(split_line(iter)));
     % accel
    iter = iter+1;
    accel = str2double(char(split_line(iter)));
    % Timestamp
    iter = iter+1;
    timestamp = str2double(char(split_line(iter)));
    
end