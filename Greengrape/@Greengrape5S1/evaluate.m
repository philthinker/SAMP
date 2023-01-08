function [results] = evaluate(obj, ExpData, NOTE)
%evaluate Evaluate the experiment results
%   - Run: 'Greengrape5S1.readCSV(...)' to load the ExpData struct array.
%   ExpData: 1 x M struct, the experiment data of each episode.
%   NOTE: String, the notes for the experiment. (Optional)
%   -------------------------------------------------
%   results: struct, the performance evaluation results with model params.
%   @Greengrape5S1

M = length(ExpData);
results = constructResults(M);

results.PreAssembly = obj.PreAssembly;
results.SAMP = obj.SAMP;
if nargin > 2
    results.note = NOTE;
end

tmpA3T = zeros(1,M);
tmpMeanA3F = zeros(1,M);
tmpMaxA3F = zeros(1,M);
for i = 1:M
    tmpA3T(i) = ExpData(i).t(4);
    tmpMeanA3F(i) = ExpData(i).meanF(3);
    tmpMaxA3F(i) = ExpData(i).maxF(3);
end
results.meanA3T = mean(tmpA3T);
results.meanA3F = mean(tmpMeanA3F);
results.maxA3F = mean(tmpMaxA3F);

% Show
disp('-------- Results --------');
disp(strcat('Time consumption in A3: ', num2str(results.meanA3T)));
disp(strcat('Max external force in A3: ', num2str(results.maxA3F)));
disp(strcat('Mean external force in A3: ', num2str(results.meanA3F)));

end

function [results] = constructResults(M)
%constructResults
results = [];
results.M = M;                      % The num. of experiment episodes
results.meanA3T = [];            % Mean time consumption in A3
results.meanA3F = [];            % Mean external force in A3
results.maxA3F = [];              % Mean max external force in A3
% Model parameters
results.PreAssembly = [];
results.SAMP = [];
% Others
results.note = '';
end