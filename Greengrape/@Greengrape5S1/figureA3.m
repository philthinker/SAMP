function [] = figureA3(obj, Demos, DemosA3)
%figureA3 Figure the demo data and the SAMP.
%   Demos: 1 x M cell of 4 x 4 x N SE(3) data, the demo data.
%   DemosA3: 1 x M cell of 13 x N data or [], the demo data of assembling phase. (Optional)
%   @Greengrape5S1
if nargin < 3
    DemosA3 = [];
end
figure;
%% Demos P
for i = 1:length(Demos)
    tmpData = Demos{i}; tmpData = permute(tmpData(1:3,4,:), [1,3,2]);
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', [0.9, 0.9, 0.9]);
    hold on;
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
if ~isempty(DemosA3)
    for i = 1:length(DemosA3)
        tmpData = DemosA3{i};
        plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', [0.7, 0.7, 0.7]);
        hold on;
    end
end
%% SAMP
p1 = obj.PreAssembly.p;
scatter3(p1(1), p1(2), p1(3), 'MarkerEdgeColor', Morandi_carnation(1), 'LineWidth', 2.5);
d1 = obj.SAMP.d(:,1)/100;
quiver3(p1(1), p1(2), p1(3), d1(1), d1(2), d1(3), 'Color', Morandi_carnation(1), 'LineWidth', 2.0);
for i = 2:obj.SAMP.K
    pi = obj.SAMP.p(:,i-1);
    scatter3(pi(1), pi(2), pi(3), 'MarkerEdgeColor', Morandi_carnation(i), 'LineWidth', 2.5);
    di = obj.SAMP.d(:,i)/100;
    quiver3(pi(1), pi(2), pi(3), di(1), di(2), di(3), 'Color', Morandi_carnation(i), 'LineWidth', 2.0);
end
pg = obj.SAMP.p(:,end);
scatter3(pg(1), pg(2), pg(3), 'MarkerEdgeColor', Morandi_carnation(obj.SAMP.K+1), 'LineWidth', 2.5);
end