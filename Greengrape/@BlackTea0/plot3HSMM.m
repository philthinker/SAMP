function [] = plot3HSMM(obj,Demos)
%plot3HSMM Plot the states together with demo data in 3D view.
%   - Use it for fast visualization of HSMM.
%   - DO NOT rely on it.
%   Demos: 1 x M cell of D x N data, the demo data.
%   @BlackTea0

MD = length(Demos);

figure;
for i = 1:MD
    tmpData = Demos{i};
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', [0.5 0.5 0.5], 'LineWidth', 0.5);
    hold on;
end
for i = 1:obj.K
    plotGMM3SC(obj.Mu(:,i), obj.Sigma(:,:,i),Morandi_carnation(i), 0.6);
end
grid on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
view(3);

end

