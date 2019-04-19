xmin = 0; xmax = 0;

len = length(x);
for i = 1:len
    if z(i) == 4;
        xx = x(i);
        if xmin ==0
            xmin = xx;
        end
        if xx > xmax
            xmax = xx;
        end
        if xx < xmin
            xmin = xx;
        end
    end
end

span = xmax - xmin