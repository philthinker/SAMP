function [K, p, q, d, mod, hsmm, DemosR] = learnSAMP_GMM_HSMM(obj, Demos)
%learnSAMP_GMM_HSMM Learn the SAMP via Density-clustering and EM for GMM & HSMM.
%   Demos: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data for assembling.
%   -------------------------------------------------
%   K: Integer >=1, the num. of states.
%   p: 3 x K, the destinate positions.
%   q: 4 x K, the destinate orientations;
%   d: 3 x K, the desired directions, v or w. (unit)
%   mod: 1 x K, the mode of AMPs.
%   |   0: Default/stationary
%   |   1: Translation
%   |   2: Rotation
%   hsmm: @BlackTea1 obj., the learnd HSMM.
%   @Greengrape5S1
%
%   Haopeng Hu
%   2022.07.04

% MOD = 0;    % Using the mean velocity only.
% MOD = 1;    % Using the eigen vector only.
MOD = 2;  % Using both the eigen vector and the velocity.

%% Seperate the demo data

M = length(Demos);
DemosTR = cell(2,M);
for i = 1:M
    tmpData = Demos{i};
    tmpN = size(tmpData,2);
    tmpDataID = [tmpData; (1:tmpN); i*ones(1,tmpN)];    % p;v;q;w;ID;DemoID
    tmpLogID = all(tmpDataID(11:13,:) == 0,1);
    if sum(tmpLogID) < tmpN/10
        % If there are too few translation motion data
        tmpLogID = false(1,tmpN);
    elseif tmpN - sum(tmpLogID) < tmpN/10
        % If there are too few rotation motion data
        tmpLogID = true(1,tmpN);
    end
    DemosTR{1,i} = tmpDataID(:, tmpLogID);      % Translation
    DemosTR{2,i} = tmpDataID(:,~tmpLogID);     % Rotation
end

%% LOF based outlier detection

% Translation
tmpDataID = cell2matrix(DemosTR(1,:));  % p;v;q;w;ID;DemoID
if ~isempty(tmpDataID)
    tmpV = tmpDataID(4:6,:);
    [tmpLOF, ~] = iceLOF(tmpV, round(size(tmpV,2)/obj.params_denoLOF(1)));          % Hyper-param.
    tmpDataIDT = tmpDataID(:,tmpLOF<=obj.params_thdLOF(1));                      % Hyper-param.
else
    tmpDataIDT = [];
end

% Rotation
tmpDataID = cell2matrix(DemosTR(2,:));  % p;v;q;w;ID;DemoID
if ~isempty(tmpDataID)
    tmpW = tmpDataID(11:13,:);
    [tmpLOF, ~] = iceLOF(tmpW, round(size(tmpW,2)/obj.params_denoLOF(2)));         % Hyper-param.
    tmpDataIDR = tmpDataID(:,tmpLOF<=obj.params_thdLOF(2));                        % Hyper-param.
else
    tmpDataIDR = [];
end

%% Density clustering & get rid of the too small cluster

% Translation
if ~isempty(tmpDataIDT)
    [tmpVK, tmpKT] = iceDensityClustering(tmpDataIDT(4:6,:), obj.params_thdDClustering(1));  % Hyper-param. 
    
    Vs = zeros(3, tmpKT);
    tmpNDT = tmpKT;
    
    tmpN = size(tmpVK,2);
    tmpDataIDDT = [tmpDataIDT; zeros(1, tmpN)]; % p;v;q;w;ID;CellID;MPID
    for i = 1:tmpKT
        if sum(tmpVK(end,:) == i) >= tmpN/obj.params_denoDClustering(1)     % Hyper-param. 
            tmpD = mean(tmpVK(1:3,tmpVK(end,:) == i),2);
            tmpD = tmpD/norm(tmpD);
            Vs(:,i) = tmpD;
            tmpDataIDDT(end, tmpVK(end,:) == i) = i;
        else
            tmpNDT = tmpNDT - 1;
        end
    end
    DataIDT = tmpDataIDDT(:, tmpDataIDDT(end,:) ~= 0);
else
    tmpNDT = 0;
    DataIDT = [];
    Vs = [];
end

% Rotation
if ~isempty(tmpDataIDR)
    [tmpVK, tmpKT] = iceDensityClustering(tmpDataIDR(11:13,:), obj.params_thdDClustering(2));  % Hyper-param.
    
    Ws = zeros(3, tmpKT);
    tmpNDR = tmpKT;
    
    tmpN = size(tmpVK,2);
    tmpDataIDDR = [tmpDataIDR; zeros(1, tmpN)]; % p;v;q;w;ID;CellID;MPID
    for i = 1:tmpKT
        if sum(tmpVK(end,:) == i) >= tmpN/obj.params_denoDClustering(2)                                   % Hyper-param.
            tmpD = mean(tmpVK(1:3,tmpVK(end,:) == i),2);
            tmpD = tmpD/norm(tmpD);
            Ws(:,i) = tmpD;
            tmpDataIDDR(end, tmpVK(end,:) == i) = i;
        else
            tmpNDR = tmpNDR - 1;
        end
    end
    DataIDR = tmpDataIDDR(:, tmpDataIDDR(end,:) ~= 0);
else
    tmpNDR = 0;
    DataIDR = [];
    Ws = [];
end

K = tmpNDT + tmpNDR;
disp('Density clustering done.');

%% The total number of MPs.

% Memory allocation
p = zeros(3,K);
q = repmat([1 0 0 0]',[1,K]);
d = zeros(3,K);
mod = zeros(1,K);   % 0: Default; 1: Translation; 2: Rotation


% Assign SAMP.mod and SAMP.d
% Translation
if tmpNDT > 0
    [tmpSSeq, tmpSSeqSort] = Greengrape5S1.stateSeqRegulate(DataIDT(end,:));
    tmpCounter = size(tmpSSeq,2);
    mod(1:tmpCounter) = 1;
    DataIDST = DataIDT; DataIDST(end,:) = tmpSSeqSort;  % p;v;q;w;ID;CellID;MPID
    if MOD == 1
        % Using the eigen vector only.
        d(:,1:tmpCounter) = translationDirection_EigenVector(DataIDST, Vs(:,tmpSSeq));
    elseif MOD == 2
        % Using both the eigen vector and the velocity.
        d(:,1:tmpCounter) = translationDirection_EigenVector(DataIDST, Vs(:,tmpSSeq), 1);
    else
        % Using the mean velocity only.
        d(:,1:tmpCounter) = Vs(:,tmpSSeq);
    end
else
    DataIDST = 0;
    tmpCounter = 0;
end
% Rotation
if tmpNDR > 0
    [tmpSSeq, tmpSSeqSort] = Greengrape5S1.stateSeqRegulate(DataIDR(end,:));
    d(:,tmpCounter+1:end) = Ws(:,tmpSSeq);
    mod(tmpCounter+1:end) = 2;
    DataIDSR = DataIDR; DataIDSR(end,:) = tmpSSeqSort + max(DataIDST(end,:));  % p;v;q;w;ID;CellID;MPID
end


%% Data re-design & EM for HSMM to find p, q. (@BlackTea1)

% Data merge
DemosR = cell(1,M);
DemosRHSMM = cell(1,M);
for i = 1:M
    if tmpNDT > 0
        tmpDataIDS_Ti = DataIDST(:, DataIDST(15,:) == i);   % p;v;q;w;ID;CellID;MPID
    else
        tmpDataIDS_Ti = [];
    end
    if tmpNDR > 0
        tmpDataIDS_Ri = DataIDSR(:, DataIDSR(15,:) == i);
    else
        tmpDataIDS_Ri = [];
    end
    tmpDataIDSi = [tmpDataIDS_Ti, tmpDataIDS_Ri];
    [~, tmpI] = sort(tmpDataIDSi(14,:));
    DemosR{i} = tmpDataIDSi([(1:13),16], tmpI);
    DemosRHSMM{i} = tmpDataIDSi(end,:);
end

hsmm = BlackTea1(K,1);
hsmm.Mu = (1:K);
hsmm.Sigma = repmat(1e-6,[1,1,K]);
hsmm = hsmm.initTransUniform();
hsmm = hsmm.learnHMM(DemosRHSMM);
hsmm = hsmm.transRegulate();

%% Terminal point of each AMP

[~, s, ~] = hsmm.FW_autoTermine3();
s = Greengrape5S1.stateSeqRegulate(s);
d = d(:,s);
mod = mod(:,s);
tmpPK = zeros(3,M,K);
tmpQK = repmat([1 0 0 0]', [1, M, K]);
for i = 1:M
    tmpData = DemosR{i};    % p;v;q;w;MPID
    tmpS = tmpData(end,1);
    j = 1;
    k = 1;
    while j < size(tmpData,2)
        if tmpData(end,j+1) ~= tmpS
            tmpPK(:,i,k) = tmpData(1:3,j);
            tmpQK(:,i,k) = tmpData(7:10,j);
            k = k + 1;
            tmpS = tmpData(end,j+1);
        end
        j = j + 1;
    end
end

for i = 1:K-1
    p(:,i) = LOFmean(tmpPK(:,:,i),1);
    q(:,i) = quatAverage(tmpQK(:,:,i));
end

end

function [DataOut] = LOFmean(Data, THD)
%LOFmean
    lof = iceLOF(Data,round(size(Data,2)/2));
    DataIn = Data(:, lof < THD);
    if isempty(DataIn)
        DataOut = mean(Data,2);
    else
        DataOut = mean(DataIn, 2);
    end
end

function [d] = translationDirection_EigenVector(DataIDST, Vs, mergeInd)
%translationDirection_EigenVector
%   DataIDST: 16 x N, [p;v;q;w;ID;CellID;MPID
%   Vs: 3 x K, the desired velocity directions
%   mergeInd: Integer, 0 for no merge, 1 for averaging. (Default: 0)
%   --------------------------------------------------
%   d: 3 x K, the deisred directions
if nargin < 3
    mergeInd = 0;
end
K = size(Vs,2);
d = zeros(3,K);
for k = 1:K
    % Extract the data set
    tmpP = DataIDST(1:3, DataIDST(end,:) == k);
    % Calculate the co-variance
    Mu = mean(tmpP,2);
    Sigma = (tmpP * tmpP')/size(tmpP,2) - Mu * Mu' + 1e-6;
    % Find the right eigen vector
    [V,~] = eig(Sigma);
    tmpCosTheta = zeros(1,3);
    for i = 1:3
        tmpCosTheta(i) = V(:,i)'*Vs(:,k);
    end
    [~, j] = max(abs(tmpCosTheta));
    tmpD = tmpCosTheta(j) * V(:,j);
    if mergeInd == 1
        % Averaging
        tmpBarD = tmpD + Vs(:,k);
        d(:,k) = tmpBarD/norm(tmpBarD);
    else
        % No merge
        d(:,k) = tmpD/norm(tmpD);
    end
end
end

