function [] = figureA123(obj, Demos, DemosA1, DemosA3, l)
%figureA123 Figure the whole model.
%   Demos: 1 x M cell of 4 x 4 x N SE(3), the demo data.
%   DemosA1: 1 x M cell of 13 x N, [p;v;q;w] data. (Optional)
%   DemosA3: 1 x M cell of 13 x N, [p;v;q;w] data. (Optional)
%   l: Scalar >0, length of the direction arrow. (Default: 0.05);
%   @Greengrape5S1

% arrowMod = 0; % Using quiver3
arrowMod = 1; % Using  mArrow3SC

if nargin < 5
    l = 0.05;
end

figure;

%% Demos P
if nargin < 4 || isempty(DemosA1) || isempty(DemosA3)
    % The whole SE(3) demo data.
    for i = 1:length(Demos)
        tmpData = Demos{i}; tmpData = permute(tmpData(1:3,4,:), [1,3,2]);
        plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', [0.8, 0.8, 0.8]);
        hold on;
    end
else
    % A1 and A3 demo data
    for i = 1:length(DemosA1)
        tmpData = DemosA1{i};
        plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', [0.7, 0.7, 0.7]);
        hold on;
    end
    for i = 1:length(DemosA3)
        tmpData = DemosA3{i};
        plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:),'--', 'Color', [0.81, 0.81, 0.81]);
        hold on;
    end
end
grid on; axis equal;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');

%% Pre-Assembly
p0 = obj.PreAssembly.p0;
p = obj.PreAssembly.p;
d = obj.PreAssembly.d * l;
scatter3(p0(1), p0(2), p0(3), 80, Morandi_violet(1), 'filled');
%     'MarkerEdgeColor', Morandi_violet(1), 'MarkerFaceColor', Morandi_violet(1), 'LineWidth', 3.0);
scatter3(p(1), p(2), p(3), 80, Morandi_violet(2), 'filled');
%     'MarkerEdgeColor', Morandi_violet(2), 'MarkerFaceColor', Morandi_violet(2), 'LineWidth', 3.0);
p0p = [linspace(p0(1), p(1), 20); linspace(p0(2), p(2), 20); linspace(p0(3), p(3), 20)];
plot3(p0p(1,:), p0p(2,:), p0p(3,:), '--', 'Color', Morandi_violet(1), 'LineWidth', 2.5);
if arrowMod == 1
    mArrow3SC(p, p+d,...
        'color', Morandi_violet(2));
else
    quiver3(p(1), p(2), p(3), d(1), d(2), d(3),...
        'Color', Morandi_violet(2), 'LineWidth', 5.5);
end

%% SAMP
ps = obj.SAMP.p;
ds = obj.SAMP.d * l;
K = obj.SAMP.K;
mod = obj.SAMP.mod;
for i = 1:K
    if mod(i) == 1
        % Translation
        ColorID = 5;
    elseif mod(i) == 2
        % Rotation
        ColorID = 6;
    end
    scatter3(ps(1,i), ps(2,i), ps(3,i), 80,Morandi_violet(ColorID),'filled');   % x y z s c
    ps0 = ps(:,i) - ds(:,i);
    if arrowMod == 1
        mArrow3SC(ps0, ps0+ds(:,i), 'color', Morandi_violet(ColorID));
    else
        quiver3(ps0(1), ps0(2), ps0(3), ds(1,i), ds(2,i), ds(3,i), 'Color', Morandi_violet(ColorID), 'LineWidth', 4.5);
    end
end
view(60,20);

end