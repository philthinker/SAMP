%main_A
%   - Just learn the policies.
%
%   Haopeng Hu
%   2022.07.01
%   All rights reserved
%   
% - Load: 'Data\main_A.mat'

modelA = Greengrape5S1();

%%%% Hyper-parameters %%%%
modelA = modelA.setParams('thdTransOrRot', 0.002);
modelA = modelA.setParams('thdRepeDist', [3e-4, deg2rad(2.0)]);
modelA = modelA.setParams('thdDClustering', [0.03, 0.8]);
%%%%%%%%%%%%%%%%%%

[modelA, DemosA1, DemosA3] = modelA.learnPreAssembly(Demos, FPS);  % Time-consuming

modelA = modelA.learnSAMP(DemosA3, DemosA1);  % Time-consuming

modelA.figureA123([], DemosA1, DemosA3, 0.01);

