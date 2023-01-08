function [idList, Mu] = kMeans(obj,Data)
%kMeans The K-Means clustering algorithm
%   Data:D x (N * M), data vectors
%   @HSMMZero

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

% Criterion to stop the EM iterative update
cumdist_threshold = 1e-10;
maxIter = 100;

% Initialization of the parameters
[~, nbData] = size(Data);

nbStates = obj.K;

cumdist_old = -realmax;
nbStep = 0;

idTmp = randperm(nbData);
Mu = Data(:,idTmp(1:nbStates));

%k-means iterations
while 1
	%E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for i=1:nbStates
		%Compute distances
		distTmp(:,i) = sum((Data-repmat(Mu(:,i),1,nbData)).^2, 1);
	end
	[vTmp,idList] = min(distTmp,[],2);
	cumdist = sum(vTmp);
	%M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for i=1:nbStates
		%Update the centers
		Mu(:,i) = mean(Data(:,idList==i),2);
	end
	%Stopping criterion %%%%%%%%%%%%%%%%%%%%
	if abs(cumdist-cumdist_old) < cumdist_threshold
		break;
	end
	cumdist_old = cumdist;
	nbStep = nbStep+1;
	if nbStep>maxIter
		disp(['Maximum number of kmeans iterations, ' num2str(maxIter) 'is reached']);
		break;
	end
end

end

