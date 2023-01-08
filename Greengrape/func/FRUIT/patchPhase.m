function [] = patchPhase(t, s, lowerBound, upperBound, alpha)
%patchPhase Patch the phase with colored patch.
%   t: 1 x N, the time series.
%   s: 1 x N Integer, the phase series.
%   lowerBound: Scalar, lower bound of the patch.
%   upperBound: Scalar, upper bound of the patch.
%   alpha: Scalar >0, the alpha. (Default: 0.5)

if nargin < 5
    alpha = 0.5;
end

N = length(t);
s0 = s(1);
NP = 1;
cPoints = ones(1,N);
for i = 1:N
    if s(i) ~= s0
        s0 = s(i);
        NP = NP + 1;
        cPoints(NP) = i;
    end
end
cPoints = cPoints(1:NP);

v = zeros(NP*4, 2);
% f = zeros(NP, 4);
c = zeros(NP, 3);

vCounter = 1;
% fCounter = 1;
for i = 1:NP-1
    tmpL = t(cPoints(i));
    tmpR = t(cPoints(i+1));
    v(vCounter:vCounter+3, :) = [tmpL, lowerBound; tmpR, lowerBound; tmpR, upperBound; tmpL, upperBound];
    vCounter = vCounter+4;
%     f(i,:) = (fCounter:fCounter+3);
%     fCounter = fCounter + 4;
    c(i,:) = Morandi_carnation(i);
end
tmpL = t(cPoints(NP));
tmpR = t(end);
v(vCounter:vCounter+3, :) = [tmpL, lowerBound; tmpR, lowerBound; tmpR, upperBound; tmpL, upperBound];
% f(NP,:) = (fCounter:fCounter+3);
c(NP,:) = Morandi_carnation(NP);

% patch('Faces',f,'Vertices',v,'FaceColor',c,'FaceAlpha',alpha,'LineStyle','none');
vCounter = 1;
for i = 1:NP
    patch('Faces',[1,2,3,4], 'Vertices',v(vCounter:vCounter+3,:), 'FaceColor', c(i,:), 'FaceAlpha',alpha,'LineStyle','none');
    vCounter = vCounter + 4;
    hold on;
end

end

