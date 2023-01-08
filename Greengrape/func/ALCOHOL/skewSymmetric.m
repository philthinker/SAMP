function [WOut] = skewSymmetric(wIn)
%skewSymmetric Transform the vector to its skew-symmetric form (so(3))
%   wIn: 3 x N, the vector/axis.
%   -------------------------------------------------
%   Wout: 3 x 3 x N, the skew-symmetric matrices

wIn = wIn(1:3,:);
N = size(wIn,2);
WOut = zeros(3,3,N);
WOut(1,2,:) = -permute(wIn(3,:),[1,3,2]);
WOut(1,3,:) = permute(wIn(2,:),[1,3,2]);
WOut(2,3,:) = - permute(wIn(1,:),[1,3,2]);
WOut(2,1,:) = permute(wIn(3,:),[1,3,2]);
WOut(3,1,:) = -permute(wIn(2,:),[1,3,2]);
WOut(3,2,:) = permute(wIn(1,:),[1,3,2]);

end

