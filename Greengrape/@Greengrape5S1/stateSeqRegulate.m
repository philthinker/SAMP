function [stateSeq, statesSeqReg] = stateSeqRegulate(statesSeq)
%stateSeqRegulate Compress the state sequence into the simple form.
%   statesSeq: 1 x N, the states sequence.
%   -------------------------------------------------
%   stateSeq: 1 x K, the regulated state sequence.
%   statesSeqReg: 1 x N, the regulated states sequence (1~K).
%   @Greengrape5S1

%% If there is only one element
if size(statesSeq,2) < 2
    stateSeq = statesSeq;
    statesSeqReg = 1;
    return;
end

%% There are more than one elements
statesSeqSort = sort(statesSeq);
stateSeq = statesSeqSort;
tmpStateID = statesSeqSort(1);
i = 2;
j = 1;
while i <= length(statesSeqSort)
    if statesSeqSort(i) ~= tmpStateID
        j = j + 1;
        stateSeq(j) = statesSeqSort(i);
        tmpStateID = statesSeqSort(i);
    end
    i = i + 1;
end
stateSeq = stateSeq(:,1:j);

%% Regulate the input states sequence to make it ascending from 1 to K
statesSeqReg = statesSeq;
for i = 1:j
    statesSeqReg(statesSeq(end,:) == stateSeq(i)) = i;
end

end