function accu = nb_null(t)
% returns the number of zeros in t 
n = size(t,2);
accu = 0;

for i=1:n
    if t(i)~=0
        accu=accu+1;
    end
end

end