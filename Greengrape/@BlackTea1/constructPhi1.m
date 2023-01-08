function [Phi1,Phi0] = constructPhi1(obj,N,dt)
%constructPhi1 Construct the Phi matrix for trajectory-HMM
%   N: integer, num. of data required
%   dt: scalar, time difference (optional)
%   -------------------------------------------------
%   Phi1: N*3*DP x N*DP, Phi matrix for DP-Dim data
%   Phi0: N*3 x N, Phi matrix for 1-Dim data
%   @BlackTea1

if nargin < 3
    dt = obj.dt;
end
DD = 3;           % Order of differentiation (1 for position, 2 for velocity, 3 for accleration)
DP = obj.D;     % Dim. of position

phi = zeros(DD);    % Basic transformation for 1 1-D data
phi(1,end) = 1;     % x(T) = x(T), dx(T) = (x(T) - x(T-1))/dt
for i = 2:DD
    % Pascal's Triangle
    phi(i,:) = (phi(i-1,:) - circshift(phi(i-1,:),[0,-1])) / dt;
end

% Transformation for N 1-D data
Phi0 = zeros(N*DD,N+DD-1); % Note that there are DD-1 more col.
for i = 1:N
    Phi0((i-1)*DD+1:i*DD,i:i+DD-1) = phi;
end
Phi0 = Phi0(:,DD:end);  % There are DD-1 more col., get rid of them.
% Handle the border of Phi0
for i = 1:DD-1          % Block
    for j = i+1:DD      % Row
        for k = 1:i     % Column
            Phi0((i-1)*DD+j,k) = 0; % The initial vel., acc. and jer. are zero
        end
    end
end
Phi1 = kron(Phi0,eye(DP));    % Transformation for N DP-D data

end

