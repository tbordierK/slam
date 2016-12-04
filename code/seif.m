
fid = fopen('../data/mit_killian/mit-killian.relations');

tline = fgets(fid);
while ischar(tline)
    disp(tline)
    tline = fgets(fid);
end

fclose(fid);


%%


fid = fopen('../data/mit_killian/mit-killian.clf');

tline = fgets(fid);
k = 0;
while k<11
    disp(tline)
    tline = fgets(fid);
    k = k+1;
end

fclose(fid);


