classdef BlackTea0 < HSMMZero
    %BlackTea0 HSMM+GMM with LQT behavior for MoCap data
    %   - UNCORK 'HiddenMarkovModel' first
    %   - It is designed for 3D position motion primarily
    %   Good luck!
    %
    %   Haopeng Hu
    %   2021.03.30
    %   All rights reserved
    %
    %   BARTENDING
    
    properties (Access = protected)
        params_minProb = 1e-6;          % The min. probability allowed.
        r = 1e-3;                                  % The LQR/LQT factor
    end
    
    methods
        function obj = BlackTea0(K, D)
            %BlackTea0 Initialize without any arguments
            %   K: Integer > 1, the num. of states.
            %   D: Integer > 1, the dim. of states.
            %   Good luck!
            if nargin < 2
                D = 6;
            end
            obj = obj@HSMMZero(D,K);
            obj.minSigmaPd = 5;
        end
        function [obj, ExpOut] = Go(obj, Demos,p0,dt)
            %Go The overall work process
            %   - It is just used for typical BlackTea0 usage and updated frequently.
            %   - Param. are adjusted for some specific usage.
            %   - DO NOT rely on it.
            %   Demos: 1 x M cell of D x N data, the demo data.
            %   p0: D x 1, the initial position
            %   dt: Scalar > 0, the time difference
            %   -----------------------------------------
            %   obj: the obj reference
            %   ExpOut: struct, the outcome
            %   |   h: K x N, the likelihood of each state
            %   |   s: 1 x N, the state seqence
            %   |   traj: 1 x N, the LQT motion trajectory
            %% Model learning
            obj = obj.initHMMKmeans(Demos);
            obj = obj.initTransUniform();
            obj = obj.learnHMM(Demos);
            obj = obj.transRegulate();
            %% Generate state sequence
            ExpOut = [];
            ExpOut.h = []; ExpOut.s = [];
            [ExpOut.h, ExpOut.s] = obj.FW_autoTermine();
            ExpOut.traj = obj.LQTIterative(p0,ExpOut.s,dt);
        end
        function obj = transRegulate(obj)
            %transRegulate Regulate the obj.trans property.
            %   Run it after obj.learnHSMM(...).
            tmpTrans = obj.Trans;
            for i = 1:obj.K
                % Clear the self-transition probability.
                tmpTrans(i,i) = 0.0;
                for j = 1:obj.K
                    % Clear the too small transition probability.
                    if tmpTrans(i,j) < obj.params_minProb
                        tmpTrans(i,j) = 0.0;
                    end
                end
                % Normalize the transition probability
                if all(tmpTrans(i,:) < obj.params_minProb)
                    tmpTrans(i,:) = 0;
                else
                    tmpTrans(i,:) = tmpTrans(i,:)/sum(tmpTrans(i,:));
                end
                obj.Trans = tmpTrans;
            end
        end
        function obj = setR(obj, r)
            %setR Set the R of the LQT controller.
            %   We always assume that the R matrix is diagonal.
            %   r: scalar > 0, the entries of r
            obj.r = max([0,r]);
        end
    end
    
    methods (Access = public)
        %% HSMM functions
        [obj] = setSigma(obj,Sigmas,IDs);
        %% Forward algorithm
        [h,s] = FW_standard(obj,N);             % Absolute duration assignment
        [h,s,H] = FW_autoTermine(obj);       % Automatically termined FW
        %% LQT controller
        [traj, u] = LQTBatch(obj,p0,s,dt);
        [traj, u] = LQTIterative(obj,p0,s,dt);
    end
    
    methods (Access = public)
        %% Fast figure
        [] = plot3HSMM(obj,Demos);
        [] = patchHSMM(obj,h);
    end
    
    methods (Access = protected)
        %% Auxiliary functions
        [Pd, ND] = durationAssign_Abs(obj,N);           % Not recommended
        function [DataOut] = probTHDCut(obj, DataIn)
            %THDCut Get rid of the low probabilities
            %   DataIn: Data input
            %   -----------------------------------------
            %   DataOut: Data output
            THD = obj.params_minProb;
            DataOut = DataIn;
            DataOut(DataIn < THD) = 0.0;
        end
    end
    
    methods (Access = public, Hidden = true)
        [h,s] = reconstructStSeq_StandardFW(obj, N);
        [h,s] = reconstructStSeq_FastFW(obj,N);
        [h,s] = reconstructStSeq_StochasticSamp(obj, N);
    end
    
    methods (Access = protected, Hidden = true)
        [Pd, NGen] = durationAssign_Uniform(obj,N,c);
    end
end

