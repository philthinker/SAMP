function [] = plotVariances(T, Mu, Sigma, Color, LineWidth, Alpha)
%plotVarianceSC Plot the variance of 1D data.
%   time: 1 x N, the time series
%   Mu: 1 x N, the centers
%   Sigma: 1 x N, the variance
%   Color: 3 x N or 3 x 1, the RGB
%   LineWidth: Scalar, the line width of centers (default: 2.0)
%   Alpha: Scalar, the alpha value of patch (default: 0.5)

N = size(T,2);
if N < 2
    return;
end

if nargin < 6
    Alpha = 0.5;
end
if nargin < 5
    LineWidth = 2.0;
end

if size(Color,2) == 1
    Color = repmat(Color, [1,N]);
end

for i = 2:N
    X = [T(i-1), T(i), T(i), T(i-1)];
    tmpMu = Mu(i-1);
    tmpSigma = Sigma(i-1);
    Y = [tmpMu-tmpSigma, tmpMu-tmpSigma, tmpMu+tmpSigma, tmpMu+tmpSigma];
    patch(X,Y,Color(:,i)','FaceAlpha',Alpha,'LineStyle','none');
    hold on;
    plot(X(1:2), repmat(tmpMu,[1,2]), 'Color', Color(:,i)', 'LineWidth', LineWidth);
end

end

