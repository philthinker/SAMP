function [traj] = trajGen_LQT(obj,p0,s,mod)
%trajGen_LQT Trajectory generation via LQT.
%   obj.
%   p0: D x 1, the initial position
%   s: 1 x N, the state series
%   mod: Integer, 0 for iterative LQT, 1 for batch LQT
%   (default: 0)
%   -----------------------------------------
%   traj: 3*D x N, the trajectory generated (p;v;a)
%   @BlackTea1

if nargin < 4
    mod = 0;
end
p0 = p0(1:obj.D,1);
if mod == 1
    % Batch LQT
    [tmpTraj, tmpU] = obj.LQTBatch(p0,s,obj.dt);
    traj = [tmpTraj; tmpU];
else
    % Iterative LQT
    [tmpTraj, tmpU] = obj.LQTIterative(p0,s,obj.dt);
    traj = [tmpTraj; tmpU];
end
end