%main_B
%   - Just learn the policies.
%
%   Haopeng Hu
%   2022.07.27
%   All rights reserved
%   
% - Load: 'Data\main_B.mat'

modelB = Greengrape5S1();

%%%% Hyper-parameters %%%%
modelB = modelB.setParams('thdTransOrRot', 0.001);
modelB = modelB.setParams('thdRepeDist', [5e-4, deg2rad(2.0)]);
modelB = modelB.setParams('thdDClustering', [0.03, 0.85]);
%%%%%%%%%%%%%%%%%%

[modelB, DemosA1, DemosA3] = modelB.learnPreAssembly(Demos, FPS);  % Time-consuming

modelB = modelB.learnSAMP(DemosA3, DemosA1);  % Time-consuming

modelB.figureA123([], DemosA1, DemosA3, 0.01);

