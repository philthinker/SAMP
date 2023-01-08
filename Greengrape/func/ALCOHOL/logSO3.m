function [so3Data, hat_w, theta] = logSO3(SO3Data, THD)
%logSO3 Logrithmic map of SO(3) data.
%   SO3Data: 3 x 3 x N, the SO(3) data.
%   THD: Scalar >0, the threshold of too small data. (Default: 1e-6)
%   -------------------------------------------------
%   so3Data: 3 x 3 x N, the so(3) data
%   hat_w: 3 x N, rotation axis (unit)
%   theta: 1 x N, angle

if nargin < 2
    THD = 1e-6;
end

N = size(SO3Data,3);
so3Data = zeros(3,3,N);
hat_w = zeros(3,N);
theta = zeros(1,N);

for i = 1:N
    R = SO3Data(:,:,i);
    if isEye(R, THD)
        so3Data(:,:,i) =zeros(3,3);
        hat_w(:,i) = zeros(3,1);
        theta(i) = 0;
    elseif abs(trace(R)+1) <= THD
        theta(i) = pi;
        hat_w(:,i) = (1/sqrt(2*(1+R(3,3))))*[R(1,3); R(2,3); 1+R(3,3)];
        so3Data(:,:,i) = skewSymmetric(hat_w(:,i))*theta(i);
    else
        theta(i) = acos((trace(R)-1)/2);
        hat_w_wedge = (1/(2*sin(theta)))*(R - R');
        so3Data(:,:,i) = hat_w_wedge*theta(i);
        hat_w(:,i) = invSkewSymmetric(hat_w_wedge);
    end
end

end

