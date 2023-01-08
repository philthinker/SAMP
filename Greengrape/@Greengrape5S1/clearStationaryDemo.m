function [Demos] = clearStationaryDemo(obj, Demos)
%clearStationaryDemo Clear the stationary data in demo.
%   - Check the dat one by one.
%   - Get rid of the outliers.
%   Demos: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data.
%   -----------------------------------------
%   Demos: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data without zero-vel. points.
%   @Greengrape5S1

% obj.params_thdStationaryData = [0.025, deg2rad(2.5)];     % Thd of p/s & rad/s of stationary data.

%% Check the data one by one

M = length(Demos);
for i = 1:M
    tmpData = Demos{i};
    tmpN = size(tmpData,2);
    tmpLogIDs = false(1,tmpN);
    for j = 1:tmpN
        if all(tmpData(11:13,j) == 0)
            % Translation
            tmpLogIDs(j) = norm(tmpData(4:6,j)) > obj.params_thdStationaryData(1);
        else
            % Rotation
            tmpLogIDs(j) = norm(tmpData(11:13,j)) > obj.params_thdStationaryData(2);
        end
    end
    Demos{i} = tmpData(:, tmpLogIDs);
end


end

%%%% Discarded %%%%
% for i = 1:M
%     % Assign the IDs
%     tmpData = [Demos{i}; (1:size(Demos{i},2))];   % p;v;q;w;ID
%     % Divide translation & rotation motion
%     TransLogID = all(tmpData(11:13,:) == 0, 1); % 1 x N
%     % Translation motion
%     tmpVID = [tmpData(4:6,TransLogID); tmpData(14,TransLogID)];
%     tmpLogID = sqrt(diag(tmpVID(1:3,:)'*tmpVID(1:3,:))') > obj.params_thdStationaryData(1);
%     tmpVID = tmpVID(:, tmpLogID);
%     % Rotation motion
%     tmpWID = [tmpData(11:13,~TransLogID); tmpData(14,~TransLogID)];
%     tmpLogID = sqrt(diag(tmpWID(1:3,:)'*tmpWID(1:3,:))') > obj.params_thdStationaryData(2);
%     tmpWID = tmpWID(:, tmpLogID);
%     Demos{i} = tmpData(1:13, [tmpVID(end,:),tmpWID(end,:)]);
% end
%%%% Discarded %%%%