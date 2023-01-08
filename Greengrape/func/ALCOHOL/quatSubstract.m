function [w] = quatSubstract(q1,q2)
%quatSubstract Substract the quaternions to get an angular velocity.
%   q1: 4 x N, the unit quaternions. [ w x y z ]'.
%   q2: 4 x N or 4 x 1, the unit quaternions. [ w x y z ]'.
%   -------------------------------------------------
%   w: 3 x N, the angular velocities.

N = size(q1,2);
w = zeros(3,N);
if size(q2,2) < N
    q2 = repmat(q2(:,1), [1,N]);
end

dq = quatProduct(q1,quatConjugate(q2));
theta = 2*acos(dq(1,:));
for i =1:N
    if norm(dq(2:4,i)) < 1e-6
        w(:,i) = [0 0 0]';
    else
        if theta(i) >pi
            theta(i) = theta(i) - 2*pi;
        end
        w(:,i) = theta(i)*dq(2:4,i)/norm(dq(2:4,i));
    end
end

end

