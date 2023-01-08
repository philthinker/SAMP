function [vecsOut] = invSkewSymmetric(matricesIn)
%invSkewSymmetric Inverse operation of skewSymmetric().
%   matricesIn: 3 x 3 x N, the skew-symmetric matrices
%   --------------------------------------------------
%   vecsOut: 3 x N, the vectors

N = size(matricesIn,3);
vecsOut = zeros(3,N);
vecsOut(1,:) = permute( -matricesIn(2,3,:), [1,3,2]);
vecsOut(2,:) = permute( matricesIn(1,3,:), [1,3,2]);
vecsOut(3,:) = permute( -matricesIn(1,2,:), [1,3,2]);

end

