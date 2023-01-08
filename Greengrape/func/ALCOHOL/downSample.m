function [DataOut] = downSample(DataIn,Step,Dim,meanFlag)
%downSample Down sample the data for efficiency.
%   DataIn: The data to be down sampled
%   Step: Integer, the sampling step
%   Dim: Integer, the dim. of sampling
%   meanFlag: Boolean, true for return the mean value (Default: false)
%   -------------------------------------------------
%   DataOut: The sampled data

Step = max([2, round(Step)]);
Dim = max([1, round(Dim)]);
Dim = min([3, Dim]);
if nargin < 4
    meanFlag = false;
end

N = size(DataIn, Dim);
tmpIDs = (1:N);
if meanFlag
    % Calculating the mean value
    FLogiIDs = mod(tmpIDs, Step) == 1;
    LLogiIDs = mod(tmpIDs, Step) == 0;
    FIDs = tmpIDs(FLogiIDs);
    LIDs = tmpIDs(LLogiIDs);
    NLast = mod(N,Step);
    if Dim == 1
        DataOut = DataIn(FLogiIDs,:,:);
        for i = 1:length(LIDs)
            DataOut(i,:,:) = mean(DataIn(FIDs(i):LIDs(i),:,:), 1);
        end
        if NLast > 0
            DataOut(end,:,:) = mean(DataIn(end-NLast+1:end,:,:),1);
        end
    elseif Dim == 2
        DataOut = DataIn(:,FLogiIDs,:);
        for i = 1:length(LIDs)
            DataOut(:,i,:) = mean(DataIn(:,FIDs(i):LIDs(i),:), 2);
        end
        if NLast > 0
            DataOut(:,end,:) = mean(DataIn(:,end-NLast+1:end,:),2);
        end
    elseif Dim == 3
        DataOut = DataIn(:,:,FLogiIDs);
        for i = 1:length(LIDs)
            DataOut(:,:,i) = mean(DataIn(:,:,FIDs(i):LIDs(i)), 3);
        end
        if NLast > 0
            DataOut(:,:,end) = mean(DataIn(:,:,end-NLast+1:end),3);
        end
    end
else
    % Return the original data
    tmpLogiIDs = mod(tmpIDs,Step) ==1;
    if Dim == 1
        DataOut = DataIn(tmpLogiIDs,:,:);
    elseif Dim == 2
        DataOut = DataIn(:,tmpLogiIDs,:);
    elseif Dim == 3
        DataOut = DataIn(:,:,tmpLogiIDs);
    end
end

end

