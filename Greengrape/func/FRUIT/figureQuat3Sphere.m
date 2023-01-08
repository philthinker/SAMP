function [trajP] = figureQuat3Sphere(trajQ, color, width, p0, scatterFlag)
%figureQuat3Sphere Figure the unit quaternion trajectory on a sphere.
%   trajQ: 4 x N, the unit quaternions
%   color: 1 x 3, the rgb color (Default: [1 0 0])
%   width: Scalar >0, the line width (Default: 1.0)
%   p0: 3 x 1, the stationary point (Default: [1 0 0]')
%   scatterFlag: Boolean, true for using scatter3 (Default: false)
%   -------------------------------------------------
%   trajP: 3 x N, the position trajectory on the sphere.

%% Calculate trajP

if nargin < 4 || isempty(p0)
    p0 = [ 1 0 0 ]';
end
N = size(trajQ, 2);
trajP = repmat(p0,[1,N]);

for i = 1:N
    R = quat2rotm(trajQ(:,i)');
    trajP(:,i) = R * p0;
end

%% Draw the sphere

[X,Y,Z] = sphere;
o = [0 0 0];
x = [1 0 0];
y = [0 1 0];
z = [0 0 1];

figure;
surf(X,Y,Z,'EdgeColor', 0.86*ones(1,3),'FaceColor', 0.96*ones(1,3), 'FaceAlpha', 0.60);
hold on;
quiver3(o(1),o(2),o(3),x(1), x(2), x(3), 'Color', [1.0, 0.0, 0.0], 'LineWidth', 1.8);
quiver3(o(1),o(2),o(3),y(1), y(2), y(3), 'Color', [0.0, 1.0, 0.0], 'LineWidth', 1.8);
quiver3(o(1),o(2),o(3),z(1), z(2), z(3), 'Color', [0.0, 0.0, 1.0], 'LineWidth', 1.8);

%% Draw the trajectory

if nargin < 2 || isempty(color)
    color = [1.0, 0.0, 1.0];
end
if nargin < 3 || isempty(width)
    width = 1.0;
end
if nargin < 4 || isempty(scatterFlag)
    scatterFlag = false;
end

if scatterFlag
    scatter3( trajP(1,:), trajP(2,:), trajP(3,:), '.', 'MarkerEdgeColor', color, 'MarkerFaceColor', color, 'LineWidth', width);
else
    plot3( trajP(1,:), trajP(2,:), trajP(3,:), 'Color', color, 'LineWidth', width);
end

axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
view(125,30);

end