function h = plotWithErrorBar(model_series, model_error, dataSeriesName)
% This function is taken from https://it.mathworks.com/matlabcentral/answers/102220-how-do-i-place-errorbars-on-my-grouped-bar-graph-using-function-errorbar-in-matlab#answer_111566

	% Creating axes and the bar graph
	ax = axes;
	h = bar(model_series,'BarWidth',1);
	% Set color for each bar face
	% h(1).FaceColor = 'blue';
	% h(2).FaceColor = 'yellow';
	% Properties of the bar graph as required
	ax.YGrid = 'on';
	ax.GridLineStyle = '-';
	xticks(ax, 1:size(model_series, 1));
	% Naming each of the bar groups
% 	xticklabels(ax,{ 'Low', 'Middle', 'High'});
	% X and Y labels
 	xlabel ('ViewId');
 	ylabel ('Mean Error');
	% Creating a legend and placing it outside the bar plot
 	lg = legend(dataSeriesName, 'AutoUpdate','off');
	lg.Location = 'BestOutside';
	lg.Orientation = 'Horizontal';
	hold on;
	% Finding the number of groups and the number of bars in each group
	ngroups = size(model_series, 1);
	nbars = size(model_series, 2);
	% Calculating the width for each bar group
	groupwidth = min(0.8, nbars/(nbars + 1.5));
	% Set the position of each error bar in the centre of the main bar
	% Based on barweb.m by Bolu Ajiboye from MATLAB File Exchange
	for i = 1:nbars
		% Calculate center of each bar
		x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
		errorbar(x, model_series(:,i), model_error(:,i), 'k', 'linestyle', 'none');
	end
end