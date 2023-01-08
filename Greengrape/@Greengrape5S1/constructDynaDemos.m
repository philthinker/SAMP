function [Demos, DemosPV, DemosQW] = constructDynaDemos(DemosH, FPS, THD_theta, quatRegFlag)
%constructDynaDemos Construct the dynamic demos data.
%   - MATLAB Robotic System Toolbox is required.
%   DemosH: 1 x M cell of 4 x 4 x N SE(3) data, the demo data in A3.
%   FPS: Integer >0, the FPS.
%   THD_theta: Scalar >0, the threshold of minimal rotation. (Default: 0.0017)
%   quatRegFlag: boolean, true for regulate the quaternion for positive w. (Default: true)
%   -------------------------------------------------
%   Demos: 1 x M cell of 13 x N data, [p; v; q; w] the dynamic demo data.
%   |   p: 3 x N, position data in spatial/base frame.
%   |   v: 3 x N, velocity data in body frame. (Body twist)
%   |   q: 4 x N, orientation (quaternion) data in spatial/base frame.
%   |   w: 3 x N, angular velocity in body frame. (Body twist)
%   DemosPV: 1 x M cell of 6 x N data, [p;dp] data in spatial/base frame.
%   DemosQW: 1 x M cell of 7 x N data, [q,dq] data in spatial/base frame.
%   @Greengrape5S1

if nargin < 3 || isempty(THD_theta)
    THD_theta = 0.0017;
end
if nargin < 4 || isempty(quatRegFlag)
    quatRegFlag = true;
end

M = length(DemosH);
dt = 1/FPS;

Demos = cell(1,M);
DemosPV = cell(1,M);
DemosQW = cell(1,M);
for i = 1:M
    tmpData = DemosH{i};
    tmpP = permute(tmpData(1:3,4,:), [1,3,2]);
    tmpQ = quatRegulate(tform2quat(tmpData), true, quatRegFlag)';
    [tmpV, tmpW] = Greengrape5S1.calculatePureTwist(tmpData,dt,THD_theta);
    DemosPV{i} = [tmpP; tmpV];
    DemosQW{i} = [tmpQ; tmpW];
    Demos{i} = [tmpP; tmpV; tmpQ; tmpW];
end

end

