function [appTraj, P, R] = genAppTraj(obj, p0, q0, dt, psFlag, beta_p, beta_v, beta_q)
%genAppTraj Generate the approaching trajectory with a default impedance controller.
%   p0: 3 x 1, the initial position.
%   q0: 4 x 1, the initial orientation.
%   dt: Scalar >0, the time step.
%   psFlag: Boolean, true for using the pre-assembly position instead of pre-alignment position. (Default: false)
%   beta_p: Scalar >0, the beta of axis parallel to obj.PreAssembly.d.
%   beta_v: Scalar >0, the beta of axis vertical to obj.PreAssembly.d.
%   beta_q: Scalar >0, the beta of unit quaternion.
%   -----------------------------------------
%   appTraj: 7 x N, the approaching trajectory. [p;q]
%   P: 3 x 3, the potential weighting matrix.
%   R: 3 x 3 SO(3), the desired frame transformation.
%   @Greengrape5S1

if nargin < 5
    psFlag = false;
end
if nargin < 8
    beta_p = 0.5;
    beta_v = 2.0;
    beta_q = 1.0;
end
dt = max([abs(dt), 1e-3]);
appTraj = repmat([p0;q0], [1, 100000]);
%% Virtual potential
Pl = diag(abs([beta_p; beta_v; beta_v]));
R = obj.desiredFrame(obj.PreAssembly.d);
P = R * Pl * R';
%% Interpolation (Virtual potential field)
pt = p0; qt = q0;
if psFlag
    pl = obj.PreAssembly.p;
else
    pl = obj.PreAssembly.p0;
end
ql = obj.PreAssembly.q;
%% Check initial position
i = 2;
if (pl - p0)'*obj.PreAssembly.d <= 0
    % Bad initial position
    eV = norm((pl - p0) - (pl-p0)'*obj.PreAssembly.d*obj.PreAssembly.d);
    while i < size(appTraj,2)
        if (pl - pt)'*obj.PreAssembly.d > eV/4
            % Terminal condition
            break;
        else
            % Interpolation
            % % p
            pt = pt - dt * beta_v * obj.PreAssembly.d;
            % % q
            qt = quatExpMap(beta_q * dt * quatLogMap(ql, qt), qt);
            appTraj(:,i) = [pt;qt];
        end
        i = i + 1;
    end
end
%% Approaching with VPF
while i <= size(appTraj,2)
    if norm(pl - pt) <= obj.params_maxPreAssemblyErr(1)
        % Terminal condition
        break;
    else
        % Interpolation
        % % p
        pt = pt + dt * P * (pl - pt);
        % % q
        qt = quatExpMap(beta_q * dt * quatLogMap(ql, qt), qt);
        appTraj(:,i) = [pt;qt];
    end
    i = i + 1;
end
appTraj(:,i) = [pl;ql];
appTraj = appTraj(:,1:i);

end