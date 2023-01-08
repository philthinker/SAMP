function [qOut] = quatRegulate(qIn,mod,posiWFlag)
%quatRegulate Regulate the unit quaternion to positive scalar mod
%   qIn: 4 x N, quat, [qw qx qy qz]
%   mod: boolean, true for row quat. (Default: false)
%   posiWFlag: boolean, true for regulate the w to positive value. (Default: true)
%   -------------------------------------------------
%   qOut: 4 x N or N x 4, quat

if nargin < 2 || isempty(mod)
    mod = false;
end

if nargin < 3 || isempty(posiWFlag)
    posiWFlag = true;
end

qOut = qIn;
if mod
    % row quat
    if posiWFlag
        indices = qIn(:,1) < 0;
        qOut(indices,:) = -qIn(indices,:);
    end
    qOut = vNormalize(qOut,1);
else
    % column quat
    if posiWFlag
        indices = qIn(1,:) < 0;
        qOut(:,indices) = -qIn(:,indices);
    end
    qOut = vNormalize(qOut);
end

end

