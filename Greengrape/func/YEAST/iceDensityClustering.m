function [DataOut,K] = iceDensityClustering(Data,THD)
%iceDensityClustering Density clustering given a constant threshold.
%   Data: D x N, the column data.
%   THD: Scalar >0, the threshold.
%   -------------------------------------------------
%   DataOut: D+1 x N, the column data augmented its cluster ID.
%   K: Integer >0, the num. of cluster.

K = 1;
D = size(Data,1);
N = size(Data,2);
DataOut = [Data; zeros(1,N); (1:N)];
DataOut(D+1,1) = K;
THD = THD^2;

tmpdata = Data(:,1);
i = 1;
while i <= N
    % Find the nearest point in the left data.
    DataLeft = DataOut(:, DataOut(D+1,:) == 0);
    % Calculate the distances
    e = tmpdata - DataLeft(1:D, :);
    d = diag(e' * e);
    % Find the nearest one.
    [min_e, j] = min(d);
    % Cluster determination.
    if min_e >= THD
        K = K + 1;
    end
    DataOut(D+1, DataLeft(end,j)) = K;
    tmpdata = DataLeft(1:D,j);
    i = i + 1;
end

DataOut = DataOut(1:end-1,:);

end

