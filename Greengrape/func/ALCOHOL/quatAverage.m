function [qOut] = quatAverage(qIn)
%quatAverage Calculate the average of unit quaternions
%   qIn: 4 x N, the unit quaternions
%   qOut: 4 x 1, the average of the input unit quaterions

% F.L. Markley, Y. Cheng, J.L. Crassidis, Y. Oshman, Averaging quaternions, J. Guid. Control Dyn. 30 (4) (2007) 1193–1197.

C = qIn * qIn';
coeff = pca(C');
qOut = quatNormalize(coeff(:,1));

end

