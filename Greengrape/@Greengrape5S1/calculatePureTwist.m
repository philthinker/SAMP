function [V,W,WV] = calculatePureTwist(H,dt,THD_theta,mod)
%calculatePureTwist Calculate the spatial/body twist given SE(3) data.
%   H: 4 x 4 x N, SE(3) data in spatial frame.
%   dt: Scalar >0, the time difference.
%   THD_theta: Scalar >0, the threshold of minimal rotation. (Default: 0.0017)
%   mod: Integer, 0 for body twist, 1 for sptial twist. (Default: 1)
%   -------------------------------------------------
%   V: 3 x N, the linear velocity.
%   W: 3 x N, the angular velocity.
%   WV: 6 x N, the twist data [w;v].
%   @Greengrape5S1
%
%   - We assume there are only pure translation and rotation in demo.
%   - Rotations smaller than 'THD_theta' are ignored.
%   - The spatial twist is used by default.

%% Default arguments
if nargin < 4
    mod = 1;
end
if nargin < 3
    THD_theta = 0.0017;
elseif isempty(THD_theta)
    THD_theta = 0.0017;
end

N = size(H,3);
V = zeros(3,N);
W = zeros(3,N);
for i = 2:N
    Rsb = H(1:3,1:3,i);
    dH = H(:,:,i)*fastInvSE3(H(:,:,i-1));
    dR = dH(1:3,1:3);
    dp = dH(1:3,4);
    p = H(1:3,4,i);
    [~, hat_w, theta] = logSO3(dR);
    if theta <= THD_theta
        % Pure translation
        ws = 0;
        wb = 0;
        vb = dp/dt;
        vs = dp/dt;
    else
        % Pure rotation
        ws = hat_w*theta/dt;
        wb = Rsb' * ws;
        dp = cross(ws, p);
        vb = Rsb' * dp;
        vs = skewSymmetric(p)*Rsb*wb + Rsb*vb;
    end
    % Output
    if mod == 1
        % Emit sptial twist
        W(:,i) = ws;
        V(:,i) = vs;
    else
        % Emit body twist
        W(:,i) = wb;
        V(:,i) = vb;
    end
end
WV = [W;V];

end

