function [] = figureForceBar(MaxMeanF, IDs)
%figureForceBar Figure the max. mean force via bar.
%   MaxMeanF: D x M, the max., mean, ... froce of M episodes.
%   IDs: 1 x M cell of String, the episode IDs.
%   @Greengrape5S1

%% Data

M = min( [size(MaxMeanF,2), length(IDs)] );
D = size(MaxMeanF,1);
IDs = IDs(1:M);
x = categorical(IDs);
x = reordercats(x,IDs);
y = MaxMeanF(:,1:M);

%% Figure

figure;
b = bar(x, y);
for i = 1:D
    b(i).FaceColor = 'flat';
    for j = 1:M
        b(i).CData(j,:) = Morandi_carnation(i);
    end
    xtips = b(i).XEndPoints;
    ytips = b(i).YEndPoints;
    labels = string(b(i).YData);
    text(xtips,ytips,labels,'HorizontalAlignment','center',...
        'VerticalAlignment','bottom');
end
grid on;
ylabel('N');

end