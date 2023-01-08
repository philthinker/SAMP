function [rgb] = Morandi_violet(index)
%Morandi_violet Morandi palette: Popsicle
%   index: integer, the color index
%   -------------------------------------------------
%   rgb: 1 x 3, rgb color value

violet_255 = ...
    [   250, 123, 137;  % red
        117, 142, 183;  % blue
        165, 202, 210;  % green
        236, 213, 159;  % brown
        111, 95, 144;  % pruple
        252, 182, 120;  % yellow
        239, 140, 134;  % pink
        4, 76, 707;  % ocean
        5, 91, 92;    % deep green
        ];
    
violet_1 = violet_255/255;

% index = max([1,round(index)]);
index = mod(index,9);
if index == 0
    index = 9;
end

rgb = violet_1(index,:);

end

