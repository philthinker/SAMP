function [] = patchHSMM(obj,h)
%patchHSMM Draw the likelihood of each state with the timeline,
%   - Use it for fast visualization of HSMM.
%   - DO NOT rely on it.
%   h: K x N, the likelihood calculated via FW and so on.
%   @BlackTea0

N = size(h,2);
figure; hold on;
for i=1:obj.K
	patch([1, 1:N, N], [0, h(i,:), 0], Morandi_carnation(i), ...
		'linewidth', 2, 'EdgeColor', max(Morandi_carnation(i)-0.2,0), 'facealpha', .6, 'edgealpha', .6);
end
set(gca,'xtick',(10:10:N)); axis([1 N 0 1.1]);
xlabel('t');  ylabel('h');

end

