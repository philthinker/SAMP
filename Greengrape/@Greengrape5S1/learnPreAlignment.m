function [p0] = learnPreAlignment(obj, DemosA1)
%learnPreAlignment Learn the pre-alignment position.
%   DemosA1: 1 x M cell of D x N, D >=3 the demo data for approaching.
%   -----------------------------------------
%   p0: 3 x 1, the pre-alignment position.
%   @Greengrape5S1

% Calculate the lower bound of the upper bounds of all the demos projected
% to the desired direciton.
p = obj.PreAssembly.p;
d = obj.PreAssembly.d;
M = length(DemosA1);
tmpFarthestP = zeros(4, M);
for i = 1:M
    tmpData = DemosA1{i};   % p, v, q, w, r
    %%%% There is a bug! But it always works. %%%%
    tmpDataEp = p - tmpData(1:3, :);    % p
    tmpDataE = tmpDataEp'*d;   % ||e|| cos(theta)
    tmpFarthestP(end,i) = max(tmpDataE);
    tmpFarthestP(1:3,i) = p - tmpFarthestP(end,i)*d;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
[~, tmpID] = min(tmpFarthestP(end,:));
p0 = tmpFarthestP(1:3, tmpID);

end