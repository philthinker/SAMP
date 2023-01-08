function [qDist] = quatDist(q1, q2, mod)
%quatDist Calculate the distance of unit quaternions.
%   q1: 4 x N, the unit quaternions, [w x y z]'.
%   q2: 4 x N, the unit quaternions, [w x y z]'.
%   mod: Integer, 0 for norm in the tangent space, 1 for the angle difference. (Default: 0)
%   --------------------------------------------------
%   qDist: 1 x N, the distances.

% If nothing wrong occurs, there is NO difference among these methods.

if nargin < 3
    mod = 0;
end

if mod == 1
    % The angle difference
    r1 = quat2rotm(q1');
    r2 = quat2rotm(q2');
    N = size(r1,3);
    qDist = zeros(1,N);
    for i = 1:N
        [~, ~, qDist(i)] = logSO3(r1(:,:,i)*r2(:,:,i)');
    end
else
    % 2 x Norm in the tangent space
    eta = quatLogMap(quatProduct(q1,quatConjugate(q2)));
    qDist = 2 * sqrt(diag(eta'*eta))';
end

end