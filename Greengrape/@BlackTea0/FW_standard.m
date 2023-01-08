function [h,s] = FW_standard(obj,N)
%FW_standard Standard Forward computation.
%   N: Integer > 0, the num. of state required
%   -------------------------------------------------
%   h: K x N, state probabilities
%   s: 1 x N, state sequence
%   @BlackTea0

% 
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

%% Arguments and var. init.

K = obj.K;
h = zeros(K,N);
% c = zeros(1,N); % Scaling factor to avoid numerical issues

%% Duration assignment

[Pd, ND] = obj.durationAssign_Abs(N);
% disp(Pd);
% disp(ND);

% c(1) = 1; % Initialization of scaling factor
for t=1:N
	for i=1:K
		if t<=ND
            % The 1st state duration
            % Observation probability for generative purpose
			oTmp = 1; 
			h(i,t) = obj.StatePrior(i) * Pd(i,t) * oTmp;
		end
		for d=1:min(t-1,ND)
            % Observation probability for generative purpose
			oTmp = 1; 
			h(i,t) = h(i,t) + h(:,t-d)' * obj.Trans(:,i) * Pd(i,d) * oTmp;
		end
	end
% 	c(t+1) = 1/sum(h(:,t)+realmin); %Update of scaling factor
end
h = h ./ repmat(sum(h,1),K,1);
[~, s] = max(h);

end

