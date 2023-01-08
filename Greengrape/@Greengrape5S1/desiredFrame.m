function [R] = desiredFrame(d)
%desiredFrame Calculate the desired frame transformation R.
%   d: 3 x 1, the desired direction.
%   -------------------------------------------------
%   R: 3 x 3 SO(3), the desired frame transformation.
%   @Greengrape5S1

x = [1 0 0]';

if 1-abs(d'*x) < 1e-3
    R = eye(3);
    return;
end

y = cross(x,d);
y = y/norm(y);

R = [d, y, cross(d,y)];

end

