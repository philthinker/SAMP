function [Pd, ND] = durationAssign_Abs(obj,N)
%durationAssign_Abs Assign the absolute duration and probabilities given the number of data
%   N: Integer, num. of data required
%   -------------------------------------------------
%   Pd: K x ND, duration probabilities
%   ND: Integer > 0, the duration
%   @BlackTea0

N = round(max([obj.K, N(1,1)]));
obj.logFlag = false;

% Number of maximum duration step to consider in the HSMM
if N >= 2*obj.minSigmaPd * obj.K
    ND = N;
else
    ND = 2*obj.minSigmaPd;
end

%Precomputation of duration probabilities 
Pd = zeros(obj.K, ND);
if obj.logFlag
    % Log normal distribution (Discarded here)
    for i=1:obj.K
        Pd(i,:) = obj.GaussPDF( log((1:ND)), obj.MuPd(:,i), obj.SigmaPd(:,:,i) );
        %The rescaling formula below can be used to guarantee that the cumulated sum is one (to avoid the numerical issues)
        Pd(i,:) = Pd(i,:) / sum(Pd(i,:));
    end
else
    % Normal distribution
    for i=1:obj.K
        Pd(i,:) = obj.GaussPDF( (1:ND), obj.MuPd(:,i), obj.SigmaPd(:,:,i) );
        %The rescaling formula below can be used to guarantee that the cumulated sum is one (to avoid the numerical issues)
        Pd(i,:) = Pd(i,:) / sum(Pd(i,:));
    end
end

end