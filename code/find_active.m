function [Y_0,Y_plus]=find_active(bound,robot_links)


m = size(robot_links,2);
[sorted_links,order] = sort(robot_links);

Y_plus = zeros(1,bound);

for i=1:bound
    Y_plus(i) = order(i+m-bound); 
end
   
accu = 1;
for i=1:(m-bound)
    if sorted_links(i)~=0
        Y_0(accu) = order(i); 
        accu = accu + 1;
    end
end

if accu == 1
    Y_0 = [];
end
    
end