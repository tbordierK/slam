function [commands] = generate_commands()

    
    up = +0.2*ones(2,30);
    up(1,:) = 0;
    down = -0.2*ones(2,30);
    down(1,:) = 0;
    side = 0.25*ones(2,4);
    side(2,:) = 0;
    commands = [up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side,up,side,down,side];

    
end