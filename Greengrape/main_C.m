%main_C
%   - Just learn the policies.
%
%   Haopeng Hu
%   2022.07.28
%   All rights reserved
%   
% - Load: 'Data\main_B.mat'

modelC = Greengrape5S1();

%%%% Hyper-parameters %%%%
modelC = modelC.setParams('thdTransOrRot', 0.001);
modelC = modelC.setParams('thdRepeDist', [5e-4, deg2rad(2.0)]);
modelC = modelC.setParams('thdDClustering', [0.03, 0.85]);
%%%%%%%%%%%%%%%%%%

[modelC, DemosA1, DemosA3] = modelC.learnPreAssembly(Demos, FPS);  % Time-consuming

modelC = modelC.learnSAMP(DemosA3, DemosA1);  % Time-consuming

modelC.figureA123([], DemosA1, DemosA3, 0.01);

