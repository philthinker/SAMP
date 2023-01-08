function [traj, u] = LQTBatch(obj,p0,s,dt)
%LQTBatch Generate trajectory by LQT controller in batch manner
%   p0: DP x 1, the initial position
%   s: 1 x N, the state sequence
%   dt: Scalar, the time difference
%   -------------------------------------------------
%   traj: DP*2 x N, the trajectory generated
%   u: DP x N-1, the input signal
%   @BlackTea0

N = length(s);

% Dynamical System settings (discrete version)
A = kron([1, dt; 0, 1], eye(obj.D));
B = kron([0; dt], eye(obj.D));
C = kron([1, 0], eye(obj.D));
%Control cost matrix
R = eye(obj.D) * obj.r;
R = kron(eye(N-1),R);

%Build CSx and CSu matrices for batch LQR
CSu = zeros(obj.D*N, obj.D*(N-1));
CSx = kron(ones(N,1), [eye(obj.D) zeros(obj.D)]);
M = B;
for n=2:N
	id1 = (n-1)*obj.D+1:n*obj.D;
	CSx(id1,:) = CSx(id1,:) * A;
	id1 = (n-1)*obj.D+1:n*obj.D; 
	id2 = 1:(n-1)*obj.D;
	CSu(id1,id2) = C * M;
	M = [A*M(:,1:obj.D), M];
end

MuQ = reshape(obj.Mu(:,s), obj.D*N, 1); 
SigmaQ = (kron(ones(N,1), eye(obj.D)) * reshape(obj.Sigma(:,:,s), obj.D, obj.D*N)) .* kron(eye(N), ones(obj.D));

%Set matrices to compute the damped weighted least squares estimate
CSuInvSigmaQ = CSu' / SigmaQ;
Rq = CSuInvSigmaQ * CSu + R;

%Reproductions
X = [p0; zeros(obj.D,1)];
%X = [Data(:,1); zeros(obj.D,1)];
rq = CSuInvSigmaQ * (MuQ-CSx*X);
u = Rq \ rq; %Can also be computed with u = lscov(Rq, rq);
traj = reshape(CSx*X+CSu*u, obj.D, N);

end

