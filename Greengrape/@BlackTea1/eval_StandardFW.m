function [h,seq] = eval_StandardFW(obj, Data, normalFlag)
%eval_StandardFW Evaluate the data by standard forward variable.
%   Data: D x N, the data.
%   normalFlag: Boolean, true for normalized likelihood. (Default: true)
%   --------------------------------------------------
%   h: K x N, the likelihood.
%   seq: 1 x K, the state ID sequence.
%   @BlackTea1

% 
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

if nargin < 3
    normalFlag = true;
end

N = size(Data,2);

K = obj.K;
h = zeros(K, N);
c = zeros(1,N);         % Scaling factor to avoid numerical issues

[Pd, ND] = obj.durationAssign_Abs(N);

c(1) = 1; % Initialization of scaling factor
for t=1:N
	for i=1:K
		if t<=ND
            % The 1st state duration
            % Observation probability for standard HSMM
			oTmp = prod(c(1:t) .* GaussianPD(Data(:,1:t), obj.Mu(:,i), obj.Sigma(:,:,i))'); 
			h(i,t) = obj.StatePrior(i) * Pd(i,t) * oTmp;
		end
		for d=1:min(t-1,ND)
            % Observation probability for standard HSMM	
			oTmp = prod(c(t-d+1:t) .* GaussianPD(Data(:,t-d+1:t), obj.Mu(:,i), obj.Sigma(:,:,i))'); 
			h(i,t) = h(i,t) + h(:,t-d)' * obj.Trans(:,i) * Pd(i,d) * oTmp;
		end
	end
	c(t+1) = 1/sum(h(:,t)+realmin); %Update of scaling factor
end

if normalFlag
    h = h ./ repmat(sum(h,1),K,1);
end

[~, seq] = max(h);

end

