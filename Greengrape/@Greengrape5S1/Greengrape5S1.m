classdef Greengrape5S1
    %Greengrape5S1 A3 + SAMP v5S.1.1
    %   - A3: The Approaching, Aligning & Assembling framework.
    %   - SAMP: Sequential Assembly Movement Primitives.
    %
    %   - We always assume the 6D motion.
    %   
    %   Haopeng Hu
    %   2022.06.07
    %   All rights reserved
    %
    %   Take 'Greengrape/mainGrape5S1_0.m' as a demo.
    
    properties (Access = public)
        PreAssembly;
        SAMP;
    end
    
    properties (Access = private)
        %% Pre-Assembly Pose
        params_maxPreAssemblyErr = [4e-4, deg2rad(1.2)];    % Max error of p, theta
        params_thdRepeDist = [4e-4, deg2rad(1.2)];                % Thd of pass by distance
        %% SAMP
        params_thdStationaryData = [0.025, deg2rad(2.5)];     % Thd of p/s & rad/s of stationary data.
        params_thdDClustering = [0.005, 0.8];                         % Thd of density clustering.
        params_denoDClustering = [10, 10];                            % Denominator of small clusters detection
        %% Ohters
        params_thdTransOrRot = 0.001  ;                               % Thd of theta between trans. and rot.
        params_FinalPointPercent = 0.036;                             % Percent of final points in demo.
        params_denoLOF = [4, 10];                                         % Denominator of the K-LOF calculation.
        params_thdLOF = [1.25, 1.25];                                     % Thd of outlier detection.
    end
    
    methods
        function obj = Greengrape5S1(PreAssembly, SAMP)
            %Greengrape5S1 Init. the obj. without any argument.
            %   PreAssembly: struct or [], the PreAssembly struct. (Optional)
            %   SAMP: struct or [], the SAMP struct. (Optional)
            obj.PreAssembly = obj.constructPreAssembly();
            obj.SAMP = obj.constructSAMP(3);
            if nargin > 0 && ~isempty(PreAssembly)
                obj.PreAssembly = PreAssembly;
            end
            if nargin > 1 && ~isempty(SAMP)
                obj.SAMP = SAMP;
            end
        end
        
        function [obj, DemosA1, DemosA3] = learnPreAssembly(obj, Demos, FPS, quatRegFlag)
            %learnPreAssembly Learn the pre-assembly pose and direciton.
            %   Demos: 1 x M cell of 4 x 4 x N SE(3) data, the demo data.
            %   FPS: Integer>0, the FPS.
            %   quatRegFlag: boolean, true for regulate the quaternion for positive w. (Default: true)
            %   -----------------------------------------
            %   obj.
            %   DemosA1: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data for approaching.
            %   DemosA3: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data for assembling.
            if nargin < 4
                quatRegFlag = true;
            end
            if length(Demos) < 2
                return;
            end
            %% Data init.
            DemosPVQW = obj.constructDynaDemos(Demos, FPS, obj.params_thdTransOrRot, quatRegFlag);
            %% Learn the pre-assembly pose
            [obj.PreAssembly.p, obj.PreAssembly.d, obj.PreAssembly.q] =...
                obj.learnPreAssembly_Repe(DemosPVQW);   % Time-consuming. (10s)
            %% Tailor the demo data
            [DemosA1, DemosA3] = obj.tailorA1A3Demo(DemosPVQW);
            %% Estimate the pre-alignment pose
            obj.PreAssembly.p0 = obj.learnPreAlignment(DemosA1);
        end
        
        function [obj, hsmm] = learnSAMP(obj, DemosA3, DemosA1)
            %learnSAMP Learn the SAMP.
            %   DemosA3: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data for assembling.
            %   DemosA1: 1 x M cell of 13 x N, [p;v;q;w] data, the demo data for approaching.
            %   -----------------------------------------
            %   obj.
            %   hsmm: @BlackTea1 obj., the temporal model.
            
            %% Estimate the goal pose
            [pg, qg] = obj.learnGoal_FinalPoses(DemosA3);
            %% Get rid of the stationary demo data
            [DemosA3Moving] = obj.clearStationaryDemo(DemosA3);
            %% Learn the SAMP
%             [obj.SAMP.K, obj.SAMP.p, obj.SAMP.q, obj.SAMP.d, obj.SAMP.mod, hsmm] =...
%                 obj.learnSAMP_DC_HSMM(DemosA3Moving);
            [obj.SAMP.K, obj.SAMP.p, obj.SAMP.q, obj.SAMP.d, obj.SAMP.mod, hsmm] =...
                obj.learnSAMP_GMM_HSMM(DemosA3Moving);
            obj.SAMP.p(:,end) = pg;
            obj.SAMP.q(:,end) = qg;
            %% Take the desired direction of the 1st AMP as the alignment direction
            if nargin >= 3
                obj.PreAssembly.d = obj.SAMP.d(:,1);
                obj.PreAssembly.p0 = obj.learnPreAlignment(DemosA1);
            end
        end

        function [obj] = setParams(obj, Name, Value)
            %setParams Set the hyper-parameters of the obj.
            %   Name: String, the parameter name.
            %   Value: the value of the quried parameter.
            if strcmp(Name, 'thdTransOrRot')
                obj.params_thdTransOrRot = Value;
            elseif strcmp(Name, 'thdRepeDist')
                obj.params_thdRepeDist = Value;
            elseif strcmp(Name, 'thdDClustering')
                obj.params_thdDClustering = Value;
            end
        end
    end
    
    methods (Access = public)
        %% Exp.
        [DemosA1, DemosA3] = tailorA1A3Demo(obj, DemosPVQWR);
        [appTraj, P, R] = genAppTraj(obj, p0, q0, dt, psFlag, beta_p, beta_v, beta_q);
        [assTraj, DemosA3DTW, model] = genAssTraj(obj, DemosA3, N, WinDTW, K, h);
        [assTraj, DemosA3PDTW, DemosA3Eta] = genAssTraj_dualPolicy(obj, DemosA3, N, WinDTW, Ks, hs);
        [ModelData] = writeCSV(obj, FileName);
        [results] = evaluate(obj, ExpData, NOTE);
        %% Figures
        [] = figureA12(obj, Demos, appTrajs);
        [] = figureA3(obj, Demos, DemosA3);
        [] = figureA123(obj, Demos, DemosA1, DemosA3, l);
    end
    
    methods (Access = public, Static = true)
        %% Construct structs.
        function [PreAssembly] = constructPreAssembly()
            %constructPreAssembly Construct the PreAssembly struct.
            PreAssembly = [];
            PreAssembly.p0 = zeros(3,1);
            PreAssembly.p = zeros(3,1);
            PreAssembly.d = [1 0 0]';
            PreAssembly.q = [1 0 0 0]';
        end
        function [SAMP] = constructSAMP(K)
            %constructSAMP Construct the SAMP struct.
            SAMP = [];
            SAMP.K = K;
            SAMP.p = zeros(3,K);
            SAMP.q = repmat([1 0 0 0]', [1,K]);
            SAMP.d = repmat([1 0 0]', [1,K]);
            SAMP.mod = zeros(1,K);  % 0: Default; 1: Translation; 2: Rotation
        end
        function [ExpData] = constructExpData(M)
            %constructExpData Construct the experiment data struct.
            %   M: Integer >0, the num. of exp. episodes. (Default: 1)
            if nargin < 1
                M = 1;
            end
            ExpData = [];
            ExpData.N = 0;          % Num. of data
            ExpData.phase = [];   % Phase index
            ExpData.k = [];          % AMP ID
            ExpData.p = [];          % Position
            ExpData.pg = [];        % Goal position
            ExpData.q = [];          % Orientation
            ExpData.qg = [];        % Goal orientation
            ExpData.f = [];           % External force
            ExpData.m = [];         % External moment
            ExpData.FPS = [];       % FPS
            ExpData.t = zeros(1,4);               % Total/A1/A2/A3 time consumption
            ExpData.maxF = [];        % Max. external foce in the whole/A2/A3
            ExpData.maxM = [];       % Max. external moment in the whole/A2/A3
            ExpData.meanF = [];      % Mean external force in the whole/A2/A3
            ExpData.meanM = [];     % Mean external moment in the whole/A2/A3
            ExpData = repmat(ExpData,[1,M]);
        end
        %% Robotics & Control
        [V,W,WV] = calculatePureTwist(H,dt,THD_theta,mod);
        [Demos, DemosPV, DemosQW] = constructDynaDemos(DemosH, FPS, THD_theta, quatRegFlag);
        %% Others
        [stateSeq, statesSeqSort] = stateSeqRegulate(statesSeq);
        [R] = desiredFrame(d);
        [] = figureForceBar(MaxMeanF, IDs);
        [trajP] = figureQuat3Sphere(DemosA3, trajQ, color, figureFlag);
        [DemosP, DemosQ] = figureDemosH(DemosH, color, width);
        %% File operation
        [ExpData] = readCSV(FileName, FPS, version);
        [] = writeCSVTrajectory(Traj, FileName);
    end
    
    methods (Access = protected)
        %% Pre-Assembly learning
        [p, d, q] = learnPreAssembly_Repe(obj, DemosPVQW);
        [p0] = learnPreAlignment(obj, DemosA1);
        %% SAMP learning
        [pg, qg] = learnGoal_FinalPoses(obj, Demos);
        [Demos] = clearStationaryDemo(obj, Demos);
        [K, p, q, d, mod, hsmm, DemosR] = learnSAMP_DC_HSMM(obj, Demos);
        [K, p, q, d, mod, hsmm] = learnSAMP_GMM_HSMM(obj, Demos); % Recommended
    end
    
end

