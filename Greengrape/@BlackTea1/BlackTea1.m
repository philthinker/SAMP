classdef BlackTea1 < BlackTea0
    %BlackTea1 Hidden Semi-Markov Model
    %   - UNCORK 'HiddenMarkovModel'
    %   - UNCORK 'BARTENDING'
    %   - It is designed for 6D motion primarily
    %   - Refer to 'Mulberry\main_BlackTea1_lead0428.m' for a demo
    %   Good luck!
    %
    %   Haopeng Hu
    %   2021.04.28
    %   All rights reserved
    %
    %   Notations
    %   |   D: The dim..
    %   |   N: The num. of data.
    %   |   K: The num. of states.
    
    properties
        
    end
    
    methods
        function obj = BlackTea1(K,D)
            %BlackTea1 Initialize the obj with K and D.
            %   K: Integer > 1, the num. of states.
            %   D: Integer > 1, the dim. of states.
            obj = obj@BlackTea0(K,D);
        end
        [obj] = copy(obj, Params, ref);
        [obj, StateSeq] = sort(obj, StateSeq);
    end
    
    methods (Access = public)
        %% HSMM related
        [h,s,mdlParam] = FW_autoTermine3(obj);   % Recommended
        [h,seq] = eval_StandardFW(obj, Data);
        %% Trajectory generation
        [traj] = trajGen_LQT(obj,p0,s,mod);
        [traj] = trajGen_GMR(obj,s,h);
        %% Figure
        [] = plot3HSMMPQ(obj, demos);
    end
    
    methods (Access = protected)
        [Phi1,Phi0] = constructPhi1(obj,N,dt);
    end
    
    methods (Access = public, Hidden = true)
        obj = initTransLeftRight(obj,N);
        [obj, ExpOut] = Go(obj, Demos,p0,dt);
        [h,s] = FW_autoTermine(obj);
        [h,s] = FW_autoTermine2(obj);
    end
    
end

