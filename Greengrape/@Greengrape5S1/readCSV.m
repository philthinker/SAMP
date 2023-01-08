function [ExpData] = readCSV(FileName, FPS, version)
%readCSV Read the experiment data from a .csv file.
%   - run 'Greengrape5S1.constructExpData(M)' to allocate the memory in advance.
%   FileName: String, name of the file to be read.
%   FPS: Integer >0, The FPS.
%   version: Integer >0, the version ID. (Default: 1)
%   |   1: v5S.1.1
%   |   2: v5S.1.2
%   -------------------------------------------------
%   ExpData: N x 25, the experiment data
%   ExpPandaData: Struct, the experiment data.
%   @Greengrape5S1
%
%%%% v5S.1.1 %%%%%%%%%%%%%%%%%%%
%   - N x 25
%   - 1: Phase ID, 0,1,2
%   - 2: ID of SAMP, 1,2
%   - 3-18: OTEE
%   - 19-24: F/M
%%%% v5S.1.2 %%%%%%%%%%%%%%%%%%%
%   - N x 22
%   - 1: Phase ID, 0,1,2
%   - 2: ID of SAMP, 1,2
%   - 3-5: Position
%   - 6-8: Goal position
%   - 9-12: Quaternion (Eigen form)
%   - 13-16: Goal quaternion (Eigen form)
%   - 17-22: F/M
%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 3
    version = 1;
end

FileName = strcat(FileName,'.csv');
Data = readmatrix(FileName);

ExpData = Greengrape5S1.constructExpData();

%% General

ExpData.N = size(Data,1);       % Num. of data
ExpData.FPS = FPS;
dt = 1/FPS; % (s)

%% Data

if version == 1
    % v5S.1.1
    ExpData.phase = Data(:,1)';     % Phase counter
    ExpData.k = Data(:,2)';             % AMP ID
    H = fold2SE3(Data(:,3:18));
    ExpData.p = Data(:,15:17)';
    ExpData.q = quatRegulate(tform2quat(H)');
    ExpData.f = Data(:,19:21)';
    ExpData.m = Data(:,22:24)';
elseif version == 2
    % v5S.1.2
    ExpData.phase = Data(:,1)';     % Phase counter
    ExpData.k = Data(:,2)';             % AMP ID
    ExpData.p = Data(:,3:5)';
    ExpData.pg = Data(:,6:8)';
    ExpData.q = fromEigenQuat(Data(:,9:12)');
    ExpData.qg = fromEigenQuat(Data(:,13:16)');
    ExpData.f = Data(:,17:19)';
    ExpData.m = Data(:,20:22)';
end

%% Evaluation

A1LogID = (ExpData.phase == 1);
A2LogID = (ExpData.phase == 2);
A3LogID = (ExpData.phase == 3);
NA1 = sum(A1LogID);
NA2 = sum(A2LogID);
NA3 = sum(A3LogID);

ExpData.t = [ExpData.N*dt, NA1*dt, NA2*dt, NA3*dt];  % Total/A1/A2/A3 time consumption

fA2 = ExpData.f(:, A2LogID);
magfA2 = sqrt(diag(fA2'*fA2));
fA3 = ExpData.f(:, A3LogID);
magfA3 = sqrt(diag(fA3'*fA3));

mA2 = ExpData.m(:, A2LogID);
magmA2 = sqrt(diag(mA2'*mA2));
mA3 = ExpData.m(:, A3LogID);
magmA3 = sqrt(diag(mA3'*mA3));

ExpData.maxF = [max([max(magfA2), max(magfA3)]), max(magfA2), max(magfA3)];        % Max. external foce in the whole/A2/A3
ExpData.maxM = [max([max(magmA2), max(magmA3)]), max(magmA2), max(magmA3)];       % Max. external moment in the whole/A2/A3
ExpData.meanF = [mean([magfA2; magfA3]), mean(magfA2), mean(magfA3)];      % Mean external force in the whole/A2/A3
ExpData.meanM = [mean([magmA2; magmA3]), mean(magmA2), mean(magmA3)];     % Mean external moment in the whole/A2/A3

end

function [qOut] = fromEigenQuat(qIn)
%fromEigenQuat
%   qIn: 4 x N
%   -------------------------------------
%   qOut: 4 x N
xyz = qIn(1:3,:);
w = qIn(4,:);
qOut = [w;xyz];
end
