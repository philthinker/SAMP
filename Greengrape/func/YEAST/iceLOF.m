function [LOFs, outlierIDs] = iceLOF(Data, K)
%iceLOF Local outlier factor calculation.
%   Data: D x N, the data.
%   K: Integer >0, the num. of neighbors.
%   -------------------------------------------------
%   LOFs: 1 x N, the LOF of each data.
%   outlierIDs: 1 x N, the IDs of data sorted by the LOF (descend).

N = size(Data,2);
K = ceil(abs(K));
if N<K+1
    error('K must be less than the num. of data minus one!');
end

%% Calculate LRD

RD = zeros(N, K);
LRD = zeros(N, K+1);

x = Data';   % N x D
x2 = sum(x.^2,2);   % N x 1
[s,t] = sort(sqrt(repmat(x2,1,N)+repmat(x2',N,1)-2*x*x'), 2);

for i = 1:K+1
    for j = 1:K
        RD(:,j)=max(s(t(t(:,i),j+1),K), s(t(:,i),j+1));
    end
    LRD(:,i)=1./mean(RD,2);
end

%% Calculate LOF

LOFs=mean(LRD(:,2:K+1),2)./LRD(:,1);
LOFs = LOFs';
[~, outlierIDs] = sort(LOFs,'descend');

end

