function [pg, qg] = learnGoal_FinalPoses(obj, Demos)
%learnGoal_FinalPoses Learn the goal pose with the final N poses.
%   - Both Euclidean and quaternion distances are used.
%   - MATLAB Robotics System Toolbox is required.
%   Demos: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data.
%   -----------------------------------------
%   pg: 3 x 1, the goal position.
%   qg: 4 x 1, the goal orientation.
%   @Greengrape5S1

endPer = obj.params_FinalPointPercent;

%% Demo data init.
M = length(Demos);
DemosPQ = cell(1,M);
for i = 1:M
    tmpP = Demos{i}; tmpP = tmpP(1:3,:);
    tmpQ = Demos{i}; tmpQ = tmpQ(7:10,:);
    DemosPQ{i} = [tmpP; tmpQ];
end

%% Candidates & Averaging
if endPer <= 0
    CandidateData = zeros(7, M);
    for i = 1:M
        tmpData = DemosPQ{i};
        CandidateData(:,i) = tmpData(:,end);
    end
else
    endPer = max([1e-6, endPer]);
    endPer = min([0.99, endPer]);
    tmpCandidates = cell(1,M);
    for i = 1:M
        tmpData = DemosPQ{i};
        tmpN = ceil(size(tmpData,2) * endPer);
        tmpCandidates{i} = tmpData(:,end-tmpN+1:end);
    end
    CandidateData = cell2mat(tmpCandidates);
end
pg = mean(CandidateData(1:3,:), 2);
qg = quatAverage(CandidateData(4:7, :));

end