function [assTraj, DemosA3DTW, model] = genAssTraj(obj, DemosA3, N, WinDTW, K, h)
%genAssTraj Generate the trajectory in assembling phase.
%   DemosA3: 1 x M cell of 13 x N [p;v;q;w] data, the demo data in A3.
%   N; Integer >0, the num. of data in the traj. (Default: 1000)
%   WinDTW: Integer >0, the window of DTW. (Default: 30)
%   K: Integer >0, the number of kernels used. (Default: 6)
%   h: Scalar >0, the width of the kernels. (Default: 15);
%   -------------------------------------------------
%   assTraj: 7 x N, the assembling trajectory. [p;q]
%   DemosA3DTW: 1 x M cell of 7 x N [i,p;eta] data, the demo data in A3 after DTW.
%   model: the learned policy model
%   @Greengrape5S1

qa = obj.SAMP.q(:,end);
M = length(DemosA3);
if nargin < 3 || isempty(N)
    N = 1000;
end

%% DTW

if nargin < 4 || isempty(WinDTW)
    WinDTW = 30;
end
DemosA3DTW = initDemos(qa, DemosA3, WinDTW);

%% ProMP

tmpDemos = [];
tmpDemos.data = [];
tmpDemos = repmat(tmpDemos,[1,M]);
for i = 1:M
    tmpData = DemosA3DTW{i};
    tmpDemos(i).data = tmpData(2:end,:);
end

if nargin < 5 || isempty(K)
    K = 6;
end
if nargin < 6 || isempty(h)
    h = 15;
end
model = ProMPZero(6,K,h);
model = model.learnLRR(tmpDemos);
traj = model.reproduct(N);
trajP = traj(1:3,:);
trajQ = quatExpMap(traj(4:6,:),qa);

assTraj = [trajP; trajQ];

end

function [Demos] = initDemos(qa, DemosA3, DTW_Window)
%initDemos
M = length(DemosA3);
Demos = cell(1,M);
% qa = model.SAMP.q(:,end);
for i = 1:M
    tmpData = DemosA3{i};
    tmpP = tmpData(1:3,:);
    tmpQ = tmpData(7:10,:);
    tmpEta = quatLogMap(tmpQ, qa);
    Demos{i} = [tmpP; tmpEta];
end
Demos = GreengrapeDTW(Demos, DTW_Window);
for i = 1:M
    tmpData = Demos{i};
    N = size(tmpData, 2);
    Demos{i} = [linspace(0,1,N); tmpData];
end
end

function [DemosDTW] = GreengrapeDTW(Demos, Window)
%GreengrapeDTW
M = length(Demos);
tmpDemos = cell(1,M);
for i = 1:M
    tmpData = Demos{i};
    tmpDemos{i} = tmpData';
end
DemosDTW = iceDTW(tmpDemos, Window);
for i = 1:M
    tmpData = DemosDTW{i};
    DemosDTW{i} = tmpData';
end
end
