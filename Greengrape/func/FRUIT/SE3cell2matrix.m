function [SE3Data, PData, QData, SO3Data] = SE3cell2matrix(CellIn)
%SE3cell2matrix Transform the cell of SE(3) data into matrix data.
%   - MATLAB Robotics System Toolbox is required.
%   CellIn: 1 x M cell of SE(3) data.
%   -------------------------------------------------
%   SE3Data: 4 x 4 x N, SE(3) data.
%   PData: 3 x N, position data.
%   QData: 4 x N, unit quaternion data.
%   SO3Data: 3 x 3 x N, SO(3) data

M = length(CellIn);
tmpNs = zeros(1,M);
for i = 1:M
    tmpNs(i) = size(CellIn{i},3);
end
N = sum(tmpNs);
SE3Data = repmat(eye(4),[1,1,N]);

counter = 1;
for i = 1:M
%     disp(i);
%     disp(counter);
    SE3Data(:, :, counter:counter+tmpNs(i)-1) = CellIn{i};
    counter = counter + tmpNs(i);
end

SO3Data = SE3Data(1:3,1:3,:);
PData = permute(SE3Data(1:3,4,:), [1,3,2]);
QData = rotm2quat(SO3Data)';

end

