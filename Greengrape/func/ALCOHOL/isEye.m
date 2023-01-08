function [flag] = isEye(Data, THD)
%isEye Detect whether the Data is a unit matrix.
%   Data: D x D, matrix
%   THD: Scalar, threshold (Default: 1e-6)
%   -------------------------------------------------
%   flag: Boolean, true for Data is a unit matrix

if nargin < 2
    THD = 1e-6;
end

dC = size(Data,2);
dR = size(Data,1);
if dC ~= dR
    flag = false;
else
    I = eye(dR);
    tmpLogs = abs(Data - I) < THD;
    flag = all(all(tmpLogs));
end

end

