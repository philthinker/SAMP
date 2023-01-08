function [SE3Out,QPOut] = calculateRelativePose(DataIn,basePose)
%calculateRelativePose Calculate the relative pose
%   DataIn: 4 x 4 x N or 7 x N, SE(3) data or [ qw qx qy qz x y z]
%   basePose: 4 x 4 x 1 or 7 x 1, base pose (optional)
%   --------------------------------------------------
%   SE3Out: 4 x 4 x N, SE(3) data out.
%   QPOut: 7 x N, [qw qx qy qz x y z] data out.

%% mod identificaiton
mod = 0;
if size(DataIn,2) == 7
    mod = 1;
end

%% Data preparation
if mod == 0
    % SE(3) data in.
    if nargin < 2
        basePose = DataIn(1:4,1:4,end);
    end
    Data = DataIn;
elseif mod == 1
    % PQ data in.
    if nargin < 2
        basePose = pq2SE3(DataIn(end,5:7)', DataIn(end,1:4)');
    end
    Data = pq2SE3(DataIn(:,5:7)', DataIn(:,1:4)');
end

%% Data output
N = size(Data,3);
SE3Out = repmat(eye(4),[1,1,N]);
% QPOut = zeros(N,7);

for i = 1:N
    SE3Out(:,:,i) = basePose\Data(:,:,i);
end
QPOut = SE3toPQ(SE3Out,true);

end

