function [] = plot3HSMMPQ(obj, demos)
%plot3HSMMPQ Plot the 6D data.
%   - We always assume that the state vector is
%   [eta_x, eta_y, eta_z, x, y, z]'.
%   - Peter Corke's Robotics Toolbox is required.
%   demos: 1 x M cell of D x N data, the demo data
%   @BlackTea1

MuP = obj.Mu(4:6,:);
MuEta = obj.Mu(1:3,:);
MuQ = quatExpMap(MuEta);
Sigma = obj.Sigma(4:6,4:6,:);

MuSE3 = quat2tform(MuQ');

E = zeros(obj.D/2, obj.K);
for i = 1:obj.K
    E(:,i) = eig(Sigma(:,:,i));
end
l = 6 * max(E,[],'all');

figure;
for i = 1:length(demos)
    tmpData  = demos{i};
    plot3(tmpData(1,:), tmpData(2,:), tmpData(3,:), 'Color', [0.6, 0.6, 0.6], 'LineWidth', 0.5);
    hold on;
end
for i = 1:obj.K
    plotGMM3SC(MuP(:,i),Sigma(:,:,i),Morandi_carnation(i),0.8);
    MuSE3(1:3,4,i) = MuP(:,i);
    trplot(MuSE3(:,:,i),'arrow','length',l,'width',0.8,'rgb');
end
axis equal;  grid on;
xlabel('x'), ylabel('y'), zlabel('z');
view(3);

end

