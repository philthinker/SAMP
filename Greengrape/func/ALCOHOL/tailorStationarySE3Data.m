function [DataOut] = tailorStationarySE3Data(Data,THD)
%tailorStationarySE3Data Get rid of the stationary SE(3) data points.
%   - MATLAB Robtoic System Toolbox is required.
%   Data: 4 x 4 x N SE(3), the data.
%   THD: 1 x 2, the p and q thresholds.
%   -------------------------------------------------
%   DataOut: 4 x 4 x N SE(3), the tailored data.

initP = Data(1:3,4,1);
initQ = tform2quat(Data(:,:,1))';

%% Get rid of the initial stationary data points.
i = 2;
while i < size(Data,3)
    tmpH = Data(:,:,i);
    tmpP = tmpH(1:3,4);
    tmpQ = tform2quat(tmpH)';
    if norm(tmpP - initP) > THD(1) || quatDist(tmpQ, initQ, 1) > THD(2)
        break;
    end
    i = i + 1;
end

Data1 = Data(:,:,i:end);

%% Get rid of the final stationary data points.
% We keep the last data.

finalP = Data1(1:3,4,end);
finalQ = tform2quat(Data1(:,:,end))';

i = size(Data1,3) - 1;
while i > 1
    tmpH = Data1(:,:,i);
    tmpP = tmpH(1:3,4);
    tmpQ = tform2quat(tmpH)';
    if norm(tmpP - finalP) > THD(1) || quatDist(tmpQ, finalQ, 1) > THD(2)
        break;
    end
    i = i - 1;
end

DataOut = Data1(:,:,[(1:i),size(Data1,3)]);

end

