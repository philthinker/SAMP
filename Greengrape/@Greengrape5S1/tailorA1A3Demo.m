function [DemosA1, DemosA3] = tailorA1A3Demo(obj, DemosPVQWR)
%tailorA1A3Demo Seperate the demo data.
%   DemosPVQWR: 1 x M cell of D x N, D > 3, the [p;v;q;w;r] data.
%   -----------------------------------------
%   DemosA1: 1 x M cell of D x N, the demo data for approaching.
%   DemosA3: 1 x M cell of D x N, the demo data for assembling.
%   @Greengrape5S1

p = obj.PreAssembly.p;
d = obj.PreAssembly.d;
M = length(DemosPVQWR);
DemosA1 = cell(1,M);
DemosA3 = cell(1,M);
for i = 1:M
    tmpData = DemosPVQWR{i};
    if size(tmpData,2) < 2
        continue;
    end
    tmpP = tmpData(1:3,:);
    e = p - tmpP;
    j = 1;
    while j < size(e,2)
        if norm(e(:,j)) < 10*obj.params_maxPreAssemblyErr(1)
            if e(:,j)'*d < 0
                break;
            end
        end
        j = j + 1;
    end
    DemosA1{i} = tmpData(:,1:j);
    if j<size(tmpData,2)
        DemosA3{i} = tmpData(:,j+1:end);
    end
end
end