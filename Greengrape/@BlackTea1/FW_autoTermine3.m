function [h,s, mdlParam] = FW_autoTermine3(obj)
%FW_autoTermine3 Automatically termined FW algorithm with regulated HSMM.
%   -------------------------------------------------
%   h: K x N, state probabilities
%   s: 1 x N, state sequence
%   mdlParam: Struct, the modified model param.
%   |   StatePrior: 1 x K, the modified state prior
%   |   Trans: K x K, the modified transition matrix
%   |   ND: 1 x K, the calculated periods
%   @BlackTea1

%% Arguments and var. init.

MuPd = obj.MuPd;            % Mean period
SigmaPd = obj.SigmaPd;    % Co-variance of period
Trans = obj.Trans;              % Transition matrix
N = 2*ceil(sum(MuPd));
K = obj.K;
StatePrior = obj.StatePrior;

% We only need ONE initial state
% Take the most possible one as the initial state
[~, i] = max(StatePrior);
StatePrior = zeros(K,1);
StatePrior(i) = 1;

%% Precomputation of duration probabilities 

Pd = zeros(K, N);
ND = zeros(1,K);

% Normal distribution
for i=1:K
    Pd(i,:) = obj.GaussPDF( (1:N), MuPd(:,i), SigmaPd(:,:,i) );
    Pd(i,:) = Pd(i,:) / sum(Pd(i,:));
    ND(i) = sum(Pd(i,:) > 1e-9);
end

% disp(Pd);
% disp(ND);

%% Regulate the Trans.

stateAppeared = false(1,K);
[~,stateCurr] = max(obj.StatePrior);
stateAppeared(stateCurr) = true;
i = 1;
while any(Trans(stateCurr,:) ~= 0) && i <= 10*obj.K
    [~, stateNext] = max(Trans(stateCurr,:));
    if stateAppeared(stateNext)
        Trans(stateCurr,stateNext) = 0;
        if sum(Trans(stateCurr,:)) ~= 0
            Trans(stateCurr,:) = Trans(stateCurr,:)/sum(Trans(stateCurr,:));
        end
    else
        stateAppeared(stateNext) = true;
        stateCurr = stateNext;
    end
    i = i + 1;
end

% disp(Trans);

%% FW computation

h = zeros(K,N);

for t=1:N
    for i=1:K
        if t<=ND(i)
            % The 1st state duration
            % Observation probability for generative purpose
            oTmp = 1;
            h(i,t) = StatePrior(i) * Pd(i,t) * oTmp;
        end
        for d=1:min(t-1,ND(i))
            % Observation probability for generative purpose
            oTmp = 1;
            h(i,t) = h(i,t) + h(:,t-d)' * Trans(:,i) * Pd(i,d) * oTmp;
        end
    end
end

h = h ./ repmat(sum(h,1),K,1);

%% Tailor the state sequence

i = N-1;
while i > K
    tmpH = h(:,i);
    tmpH(tmpH < 1e-3) = 0.0;
    tmpH = tmpH./(sum(tmpH));
    if max(tmpH) < 1
        break;
    end
    i = i - 1;
end
N = i+1;
h = h(:,1:N);

[~, s] = max(h);
mdlParam = [];

mdlParam.StatePrior = StatePrior;
mdlParam.Trans = Trans;
mdlParam.ND = ND;

end

