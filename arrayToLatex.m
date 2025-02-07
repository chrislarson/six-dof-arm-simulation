function s = arrayToLatex(T, precision)
s = '';
if isnumeric(T)
T(abs(T)<(norm(T)/1000))=0;
end

if ~exist('precision','var')
    precision = 3;
end

for i=1:size(T,1)
    for j = 1:(size(T,2)-1)
        if isnumeric(T(i,j))
            s = [s sprintf(['%3.' int2str(precision) 'g&'],T(i,j))];
        else
            s = [s sprintf('%s&',latex(T(i,j)))];
        end
    end
    if isnumeric(T(i,end))
        s = [s sprintf(['%3.' int2str(precision) 'g'],T(i,end))];
    else
        s = [s sprintf('%s',latex(T(i,end)))];
    end
    if i ~= size(T,1)
        s = [s sprintf('\\\\')];
    end
end
end