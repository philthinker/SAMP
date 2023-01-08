function [trajP] = figureQuat3Sphere(DemosA3, trajQ, color, figureFlag)
%figureQuat3Sphere Figure the unit quaternion trajectory on a sphere.
%   DemosA3: 1 x M cell of 13 x N [p;v;q;w] data, demo data in
%   A3.(Optinoal)
%   trajQ: 4 x N, the unit quaternions.
%   color: 1 x 3, the RGB color. (Default: [1 0 0 ])
%   figureFlag: Boolean, true for draw a new figure. (Default: true)
%   -------------------------------------------------
%   trajP: 3 x N, the position trajectory on the sphere.
%   Greengrape5S1

if nargin < 3
    color = [1 0 0];
end
if nargin < 4
    figureFlag = true;
end

%% Calculate trajP, DemosP

p0 = [ 1 0 0 ]';
N = size(trajQ, 2);
trajP = repmat(p0,[1,N]);
for i = 1:N
    R = quat2rotm(trajQ(:,i)');
    trajP(:,i) = R * p0;
end

if ~isempty(DemosA3)
    DemosP = DemosA3;
    for i = 1:length(DemosA3)
        tmpData = DemosA3{i};
        N = size(tmpData,2);
        tmpP = zeros(3, N);
        for j = 1:N
            tmpP(:,j) = quat2rotm(tmpData(7:10,j)') * p0;
        end
        DemosP{i} = tmpP;
    end
end

%% Draw the sphere

if figureFlag
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
    
    axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');
    view(160,20);
end

%% Draw the demo trajectory

if ~isempty(DemosA3)
    for i = 1:length(DemosP)
        tmpData = DemosP{i};
        plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), '--', 'Color', 0.6*ones(1,3), 'LineWidth',1.5);
    end
end

%% Draw the given trajectory

width = 3.0;
plot3( trajP(1,:), trajP(2,:), trajP(3,:), 'Color', color, 'LineWidth', width);

end