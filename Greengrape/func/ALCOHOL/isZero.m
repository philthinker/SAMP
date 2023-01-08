function [Flag] = isZero(Data,THD)
%isZero True for a nearly zero matrix.
%   Data: D1 x D2, matrix.
%   THD: Scalar > 0, the threshold. (Default: 1e-12)
%   -------------------------------------------------
%   Flag: Boolean, true for a zero matrix.

if nargin < 2
    THD = 1e-12;
end

Flag = all(all(abs(Data) < THD));


end

