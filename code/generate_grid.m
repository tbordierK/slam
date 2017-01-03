function [map] = generate_grid(n_grid)

map = zeros(1,n_grid*n_grid);
k = 1;
for i=1:n_grid
    for j=1:n_grid
        map(1,k) = i;
        map(1,k+1) = j;
        k = k+2;
    end
end

end