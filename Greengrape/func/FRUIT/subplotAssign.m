function [numRow,numCol] = subplotAssign(N)
%subplotAssign Assign the num. of rows and columns of subplot.
%   N: Integer >0, num. of plots.
%   -------------------------------------------------
%   numRow: Integer >0, num. of rows.
%   numCol: Integer >0, num. of columns.

if N <= 3
    numRow = 1;
    numCol = 3;
    return;
end

if mod(sqrt(N),1) == 0
    numRow = sqrt(N);
    numCol = sqrt(N);
    return;
end



end

