function [DemosP, DemosQ] = figureDemosH(DemosH, color, width)
%figureDemosH Figure the demos data.
%   DemosH: 1 x M cell of 4 x 4 x N SE(3) data, the demo data.
%   color: 1 x 3, the RGB data. (Default: [0.8,0.8,0.8])
%   width: Scalar >0, the width. (Default: 1.0);
%   -------------------------------------------------
%   DemosP: 1 x M cell of 3 x N data, the demo position data
%   DemosQ: 1 x M cell of 4 x N data, the demo quaternion data
%   @Greengrape5S1

if nargin < 2
    color = [0.8,0.8,0.8];
end
if nargin < 3
    width = 1.0;
end

%% Data

M = length(DemosH);
DemosP = cell(1,M);
DemosQ = cell(1,M);
for i = 1:M
    tmpData = DemosH{i};
    tmpP = permute(tmpData(1:3,4,:), [1,3,2]);
    tmpQ = tform2quat(tmpData)';
    DemosP{i} = tmpP;
    DemosQ{i} = tmpQ;
end

%% Plot3 position

figure;
for i = 1:M
    tmpP = DemosP{i};
    plot3(tmpP(1,:), tmpP(2,:), tmpP(3,:), 'Color', color, 'LineWidth', width);
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(3);

%% Plot3 quaternion

% Draw the sphere
[X,Y,Z] = sphere;
o = [0 0 0];
x = [1 0 0];
y = [0 1 0];
z = [0 0 1];

figure;
surf(X,Y,Z,'EdgeColor', 0.86*ones(1,3),'FaceColor', 0.96*ones(1,3), 'FaceAlpha', 0.60);
hold on;

% Draw the axes
quiver3(o(1),o(2),o(3),x(1), x(2), x(3), 'Color', [1.0, 0.0, 0.0], 'LineWidth', 1.8);
quiver3(o(1),o(2),o(3),y(1), y(2), y(3), 'Color', [0.0, 1.0, 0.0], 'LineWidth', 1.8);
quiver3(o(1),o(2),o(3),z(1), z(2), z(3), 'Color', [0.0, 0.0, 1.0], 'LineWidth', 1.8);

% Draw the trajectory
p0 = [ 1 0 0 ]';
for i = 1:M
    tmpR = quat2rotm(DemosQ{i}');
    tmpN = size(tmpR, 3);
    tmpP = repmat(p0,[1,tmpN]);
    for j = 1:tmpN
        tmpP(:,j) = tmpR(:,:,j) * p0;
    end
    plot3(tmpP(1,:), tmpP(2,:), tmpP(3,:), 'Color', color, 'LineWidth', width);
end
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
view(125,30);

end

