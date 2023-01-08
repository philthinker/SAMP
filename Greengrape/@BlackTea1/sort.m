function [obj, StateSeq] = sort(obj, StateSeq)
%sort Sort the centers of the HSMM (given a sequence of state IDs).
%   - Attentioan! The original Trans. will be replaced!
%   StateSeq: 1 x K, a sequence of sorted state sequence. (Optional)
%   -------------------------------------------------
%   obj.
%   StateSeq: 1 x K, a sequence of sorted state sequence.
%   @BlackTea1

if nargin < 2
    % Generate a state sequence by FW.
    [~, StateSeq] = obj.FW_autoTermine3();
end

stateSeqOut = ones(1,max(StateSeq))*StateSeq(1);
counter = 1;
for i = 1:length(StateSeq)
    if StateSeq(i) ~= stateSeqOut(counter)
        counter = counter + 1;
        stateSeqOut(counter) = StateSeq(i);
    end
end
StateSeq = stateSeqOut;

obj.K = size(StateSeq,2);

obj.Mu = obj.Mu(:,StateSeq);
obj.Sigma = obj.Sigma(:,:,StateSeq);
obj.MuPd = obj.MuPd(:,StateSeq);
obj.SigmaPd = obj.SigmaPd(:,:,StateSeq);

StatePrior = zeros(1,obj.K); StatePrior(1) = 1;
obj.StatePrior = StatePrior;
obj.Prior = obj.Prior(StateSeq);

obj.Trans = zeros(obj.K, obj.K);
if obj.K > 1
    obj.Trans(1:obj.K-1,2:obj.K) = eye(obj.K-1);
    obj.Trans(obj.K, obj.K-1) = 1;
else
    obj.Trans = 1;
end

end

