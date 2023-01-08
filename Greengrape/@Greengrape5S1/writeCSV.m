function [ModelData] = writeCSV(obj, FileName)
%writeCSV Write the model into a csv file.
%   obj.
%   FileName: String, the name of file.
%   -------------------------------------------------
%   ModelData: 2+K x 11, the output data
%   @Greengrape5S1
%
%%%% v5S.1.1 %%%%%%%%%%%%%%%%%%
%   - pl', qs', dl';        % Pre-Alignment position/orientation/direction
%   - ps', 0, ;             % Pre-Assembly position
%   - mod, p', q', d';  % SAMP mod/position/orientation/direction
%   - ...
%%%%%%%%%%%%%%%%%%%%%%%%%%%

ModelData = zeros(2+obj.SAMP.K, 11);
% Pre-Assembly
ModelData(1,1:3) = obj.PreAssembly.p0';
ModelData(1,4:7) = toEigenQuat(obj.PreAssembly.q');
ModelData(1,8:10) = obj.PreAssembly.d';
ModelData(2,1:3) = obj.PreAssembly.p';
% SAMP
for i = 1:obj.SAMP.K
    ModelData(2+i, 1) = obj.SAMP.mod(i);
    ModelData(2+i,2:4) = obj.SAMP.p(:,i)';
    ModelData(2+i,5:8) = toEigenQuat(obj.SAMP.q(:,i)');
    ModelData(2+i,9:11) = obj.SAMP.d(:,i)';
end

writematrix(ModelData,strcat(FileName,'.csv'));

end

function [q] = toEigenQuat(q)
%toEigenQuat Tranform the quaternion data to its Eigen version.
%   q: N x 4, [w x y z] quat.
%   --------------------------------------------------
%   q: N x 4, [x y z w] quat.
tmpQ = q;
tmpQ(:,1:3) = q(:,2:4);
tmpQ(:,4) = q(:,1);
q = tmpQ;
end
