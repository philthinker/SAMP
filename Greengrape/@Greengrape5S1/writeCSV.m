function [ModelData] = writeCSV(obj, FileName, reguFlag)
%writeCSV Write the model into a csv file.
%   obj.
%   FileName: String, the name of file.
%   reguFlag: Boolean, ture for regulate the final pose to the origin. (Default: false)
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

if narign < 3
    reguFlag = false;
end

ModelData = zeros(2+obj.SAMP.K, 11);

if reguFlag
    % Regulate the final pose to the origin
    priorP = obj.SAMP.p(:,end);
    priorQ = obj.SAMP.q(:,end);
    priorR = quat2rotm(priorQ');
else
    priorP = zeros(3,1);
    priorQ = [1 0 0 0]';
    priorR = eye(3);
end
priorT = eye(4);
priorT(1:3,1:3) = priorR;
priorT(1:3,4) = priorP;
invPriorT = fastInvSE3(priorT);

% Pre-Assembly
ModelData(1,1:3) = SE3TransP(obj.PreAssembly.p0, invPriorT)';
ModelData(1,4:7) = toEigenQuat(quatInvProduct(priorQ, obj.PreAssembly.q)');
ModelData(1,8:10) = (priorR' * obj.PreAssembly.d)';
ModelData(2,1:3) = SE3TransP(obj.PreAssembly.p, invPriorT)';
% SAMP
for i = 1:obj.SAMP.K
    ModelData(2+i, 1) = obj.SAMP.mod(i);
    ModelData(2+i,2:4) = SE3TransP(obj.SAMP.p(:,i), invPriorT)';
    ModelData(2+i,5:8) = toEigenQuat( quatInvProduct(priorQ,obj.SAMP.q(:,i))');
    ModelData(2+i,9:11) = (priorR' * obj.SAMP.d(:,i))';
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

function [q] = quatInvProduct(q1, q2)
%quatInvProduct conj_q1 * q2
%   q1: 4 x 1, [w x y z]' quat
%   q2: 4 x 1, [w x y z]' quat
%   -------------------------------------------------
%   q: 4 x 1, [w x y z]' quat

q = quatProduct(quatConjugate(q1), q2);

end

function [p] = SE3TransP(p, T)
%SE3TransP Transform p with the SE(3) matrix T
%   p: 3 x 1, the p
%   T: 4 x 4 SE(3), the T
%   -------------------------------------------------
%   p: 3 x 1, the tranformed p

p = [p;1];
p = T * p;
p = p(1:3);

end
