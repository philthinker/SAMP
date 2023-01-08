function [] = writeCSVTrajectory(Traj, FileName)
%writeCSVTrajectory Write the model into a csv file.
%   Traj: 7 x N, the trajectory data.
%   FileName: String, the name of file.
%   -------------------------------------------------
%   @Greengrape5S1

TrajData = Traj';
TrajData(:,4:7) = toEigenQuat(TrajData(:,4:7));

writematrix(TrajData,strcat(FileName,'.csv'));

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


