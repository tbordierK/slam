function [map] = generate_grid()

map = zeros(1,10*10);
k = 1;
for i=1:10
    for j=1:10
        map(1,k) = i;
        map(1,k+1) = j;
        k = k+2;
    end
end

end