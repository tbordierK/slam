function [commands] = generate_commands()

    
    up = +0.05*ones(2,40);
    up(1,:) = 0;
    down = -0.05*ones(2,40);
    down(1,:) = 0;
    side = 0.05*ones(2,20);
    side(2,:) = 0;
    %commands = [up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side];
    commands = repmat([up,side,down,side,up,-side,-side,down/2,side,side,down/2,-side,-side],1,2);
    
end