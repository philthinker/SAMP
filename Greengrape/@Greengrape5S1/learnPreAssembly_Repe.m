function [p, d, q] = learnPreAssembly_Repe(obj, DemosPVQW)
%learnPreAssembly_Repe Learn the pre-assembly pose with 6D demo dynamic data.
%   - Both Euclidean and quaternion distances are used.
%   - The demo data are tailored herein.
%   - MATLAB Robotics System Toolbox is required.
%   DemosPVQW: 1 x M cell of D x N, D >= 10, the [p;v;q;w] data.
%   -------------------------------------------------
%   p: 3 x 1, the pre-assembly position.
%   d: 3 x 1, the pre-assembly direction.
%   q: 4 x 1, the pre-assembly orientation.
%   @Greengrape5S1

M = length(DemosPVQW);
DistTHD = obj.params_thdRepeDist(1);                 % THD of the distance
QuatTHD = obj.params_thdRepeDist(2);               % THD of the orientation distance
RepeTHD = M/2;                                                 % THD of repetitive
OutTHD = 1.0;                                                     % THD of the outlier factor

% %% Demo data init.

DemosPVQR = cell(1,M);
for i = 1:M
    tmpData = DemosPVQW{i};
    N = size(tmpData,2);
    DemosPVQR{i} = [tmpData(1:10,:); zeros(2,N)];   % p;v;q;r
end

%% Repetitive evaluation

disp('Evaluating the repetitive');
for i = 1:M
    fprintf('.');
    N = size(DemosPVQR{i}, 2);
    tmpPVQR = DemosPVQR{i};     % p;v;q;r
    for j = 1:N
        tmpPj = tmpPVQR(1:3,j);         % 3 x 1
        tmpQj = tmpPVQR(7:10,j);       % 4 x 1
        for k = 1:M
            if k ~= i
                % Euclidean distance
                tmpPk = DemosPVQW{k}; tmpPk = tmpPk(1:3, :);    % 3 x N
                tmpE = tmpPk - tmpPj;      % 3 x N
                tmpD = sqrt(diag(tmpE' * tmpE))'; % 1 x N
                if min(tmpD) <= DistTHD
                    tmpPVQR(11,j) = tmpPVQR(11,j) + 1;
                    % Quat distance
                    tmpQk = DemosPVQW{k}; tmpQk = tmpQk(7:10, tmpD<=DistTHD);   % 4 x N'
                    dQuat = quatDist(tmpQk, repmat(tmpQj,[1,size(tmpQk,2)]), 1);
                    if min(dQuat) < QuatTHD
                        tmpPVQR(12,j) = tmpPVQR(12,j) + 1;
                    end
                end
            end
        end
        if tmpPVQR(12,j) >= RepeTHD
            break;
        end
    end
    DemosPVQR{i} = tmpPVQR;
end
disp('Repeatitive checking done.');

%% Find the candidates

PCandidates = zeros(12,M); % p, v, quat, LOF, flag
tmpNotEmptyID = true(1,M);
for i = 1:M
    tmpPVQR = DemosPVQR{i};
    tmpCandidates = tmpPVQR(:, tmpPVQR(12,:) > RepeTHD);
    if ~isempty(tmpCandidates)
        PCandidates(1:10,i) = tmpCandidates(1:10,1);
    else
        tmpNotEmptyID(i) = false;
    end
end
PCandidates = PCandidates(:, tmpNotEmptyID);

%% Get rid of the outliers

tmpM = size(PCandidates,2);
% Fine the outliers by LOF
if tmpM > 2
    PCandidates(11,:) = iceLOF(PCandidates(1:3,:), tmpM-2);
end
for i = 1:tmpM
    if PCandidates(11,i) > OutTHD
        PCandidates(12,i) = 1;
    end
end

%% Final evulation of p, q

p = mean(PCandidates(1:3,  PCandidates(12,:) == 0),2);
q = quatAverage(PCandidates(7:10, PCandidates(12,:) ==0));

%% Estimate the desired direction
%{
% Method 1: In view of candidates
d = mean(PCandidates(4:6, PCandidates(12,:) ==0),2);
d = d/norm(d);
%}
%
% Method 2: In view of neighbors
% Find the neighbors
tmpDemo = cell2matrix(DemosPVQR);
tmpEp = tmpDemo(1:3,:) - p;
tmpE = sqrt(diag(tmpEp'*tmpEp))';

neighborPV = tmpDemo(1:6, tmpE <= 2*DistTHD);

% Calculate the desired direction
if size(neighborPV,2) >= 10
    tmpFlag = iceLOF(neighborPV(4:6,:), round(size(neighborPV,2)/2)+1);
    tmpV = neighborPV(4:6, tmpFlag < 1.1);
else
    tmpV = neighborPV(4:6, :);
end
d = mean(tmpV, 2); d = d/norm(d);
%}

end