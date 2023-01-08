function [obj] = copy(obj, Params, ref)
%copy Copy the parameters or reference object.
%   Params: Struct or []
%   |   Mu: D x K
%   |   Sigma: D x D x K
%   |   MuPd: 1 x K
%   |   SigmaPd:    1 x 1 x K
%   |   StatePrior: 1 x K
%   |   Trans: K x K
%   ref: @BlackTea1 obj. (Optional)
%   --------------------------------------------------
%   obj.
%   @BlackTea1

if nargin < 3
    ref = obj;
end

if ~isempty(Params)
    obj.Mu = Params.Mu;
    obj.Sigma = Params.Sigma;
    obj.MuPd = Params.MuPd;
    obj.SigmaPd = Params.SigmaPd;
    obj.StatePrior = Params.StatePrior;
    obj.Trans = Params.Trans;
else
    obj.Mu = ref.Mu;
    obj.Sigma = ref.Sigma;
    obj.MuPd = ref.MuPd;
    obj.SigmaPd = ref.SigmaPd;
    obj.StatePrior = ref.StatePrior;
    obj.Trans = ref.Trans;
end

obj.K = size(obj.Mu,2);
obj.Prior = 1/obj.K * ones(1,obj.K);

end

