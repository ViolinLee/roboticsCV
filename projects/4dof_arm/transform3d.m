function transform3d(T, mode)
% Coordinate frame transformation animation  
switch mode
    case 0
        trplot(T)
    case 1
        for i=1:length(T)
            trplot(T(i))
        end
end

end
