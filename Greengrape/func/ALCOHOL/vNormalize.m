function [DataOut] = vNormalize(DataIn, mod)
%vNormalize Normalize the vectors.
%   DataIn: D x N or N x D, the input data.
%   mod: Integer, 0 for D x N, 1 for N x D data. (Default: 0)
%   -------------------------------------------------
%   DataOut: D x N or N x D, the normalized output vectors.

if nargin < 2
    mod = 0;
end

if mod == 1
    % N x D
    DataOut = DataIn./sqrt(diag(DataIn * DataIn'));
else
    % D x N
    DataOut = DataIn./sqrt(diag(DataIn' * DataIn)');
end

end

