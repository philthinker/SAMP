function [] = figureA12(obj, Demos, appTrajs)
%figureA3 Figure the demo data, approaching trajectory and the pre-assembly pose.
%   Demos: 1 x M cell of 4 x 4 x N SE(3) data, the demo data.
%   appTrajs: 1 x M cell of 7 x N or [], the approaching trajectories. ([p;q], Optional)
%   @Greengrape5S1
if nargin < 3
    appTrajs = [];
end
figure;
%% Demos P
for i = 1:length(Demos)
    tmpData = Demos{i}; tmpData = permute(tmpData(1:3,4,:), [1,3,2]);
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', [0.88, 0.88, 0.88]);
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
%% Approaching trajectories
if ~isempty(appTrajs)
end
%% Pre-assembly
p0 = obj.PreAssembly.p0;
p = obj.PreAssembly.p;
d = obj.PreAssembly.d/100;
scatter3(p0(1), p0(2), p0(3), 'MarkerEdgeColor', Morandi_carnation(1), 'LineWidth', 2.0);
scatter3(p(1), p(2), p(3), 'MarkerEdgeColor', Morandi_carnation(2), 'LineWidth', 2.0);
p0p = [linspace(p0(1), p(1), 20); linspace(p0(2), p(2), 20); linspace(p0(3), p(3), 20)];
plot3(p0p(1,:), p0p(2,:), p0p(3,:), '--', 'Color', Morandi_carnation(1), 'LineWidth', 1.5);
quiver3(p(1), p(2), p(3), d(1), d(2), d(3), 'Color', Morandi_carnation(2), 'LineWidth', 1.5);
end