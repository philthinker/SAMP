function [obj] = setSigma(obj,Sigmas,IDs)
%setSigma Set the Sigmas. 
%   Note that only the upper-triangular entries are assigned due to the symmetric nature.
%   Sigmas: D x D x K, the Sigmas.
%   IDs: 1 x K or [], the IDs to be set.
%   -------------------------------------------------
%   obj
%   @BlackTea0

if nargin < 3 || isempty(IDs)
    IDs = (1:obj.K);
end

for k = 1:length(IDs)
    for i = 1:obj.D
        for j = i:obj.D
            obj.Sigma(i,j,IDs(k)) = Sigmas(i,j,IDs(k));
            obj.Sigma(j,i,IDs(k)) = Sigmas(i,j,IDs(k));
        end
    end
end

end

