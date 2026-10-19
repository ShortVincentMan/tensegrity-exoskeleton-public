% CH2 exploratory comparison. No toolboxes required.
% Run, select EMG_ALL.csv from the earlier package, then mark each trial.
% This is centered-signal RMS, NOT validated broadband muscle activation.
clear; clc; close all;
Fs = 51.2;
windowSamples = round(0.5*Fs);
[file,folder] = uigetfile('*.csv','Select EMG_ALL.csv (combined recordings)');
if isequal(file,0), return; end
T = readtable(fullfile(folder,file),'VariableNamingRule','preserve');
groups = ["BL_NW","AS_NW","BL_15","AS_15"];
Y = cell(4,3); onset = zeros(4,3); finish = zeros(4,3);
trialMean = zeros(4,3); names = strings(4,3);
for g = 1:4
    for r = 1:3
        name = groups(g)+"_"+r+"_CH2";
        if ~ismember(name,string(T.Properties.VariableNames))
            error('Missing column %s. Select the combined EMG_ALL.csv.',name);
        end
        x = T.(char(name));
        last = find(isfinite(x),1,'last');
        if isempty(last), error('Empty trial: %s',name); end
        x = x(1:last);
        if any(~isfinite(x)), error('Interior missing samples in %s.',name); end
        t = (0:numel(x)-1)'/Fs;
        f = figure('Color','w','Name',char(name),'Position',[100 150 1100 520]);
        plot(t,x,'Color',[0.1 0.35 0.55]); grid on;
        xlabel('Seconds from recording start (assumed uniform sampling)');
        ylabel('Raw CH2 voltage (mV)');
        title({char(name),'Click movement START, then END of comparable activity',...
            'Use notes/video to identify activity. Rest may be 0 or about 2 s; drift alone is not onset.'},...
            'Interpreter','none');
        xlim([0 t(end)]);
        [clicks,~] = ginput(2);
        if numel(clicks)~=2, error('Select two points for each trial.'); end
        idx = round(clicks*Fs)+1;
        idx = max(1,min(numel(x),idx));
        if idx(2)<=idx(1) || diff(idx)+1<windowSamples
            error('Select an end after the start, with at least 0.5 s of activity.');
        end
        onset(g,r) = t(idx(1)); finish(g,r) = t(idx(2));
        % Center each selected interval independently; no narrow bandpass.
        segment = x(idx(1):idx(2));
        segment = segment-mean(segment);
        % Full windows only: avoid shrinking RMS windows at segment edges.
        Y{g,r} = sqrt(conv(segment.^2,ones(windowSamples,1)/windowSamples,'valid'));
        names(g,r) = name;
        close(f);
    end
end
% Identical duration and exactly 3 trials at every plotted point.
N = min(cellfun(@numel,Y),[],'all');
t = ((0:N-1)' + (windowSamples-1)/2)/Fs;
mu = zeros(N,4); sd = zeros(N,4);
for g = 1:4
    A = zeros(N,3);
    for r = 1:3
        A(:,r) = Y{g,r}(1:N);
        trialMean(g,r) = mean(A(:,r));
    end
    mu(:,g) = mean(A,2); sd(:,g) = std(A,0,2);
end
fig = figure('Color','w','Position',[100 100 1200 650]);
layout = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
colors = [0.15 0.35 0.65; 0.05 0.55 0.35];
axesList = gobjects(1,2);
for panel = 1:2
    ax = nexttile; axesList(panel) = ax; hold on;
    handles = gobjects(1,2);
    for k = 1:2
        g = (panel-1)*2+k;
        lower = max(0,mu(:,g)-sd(:,g)); upper = mu(:,g)+sd(:,g);
        fill([t;flipud(t)],[lower;flipud(upper)],colors(k,:),...
            'FaceAlpha',0.15,'EdgeColor','none','HandleVisibility','off');
        handles(k) = plot(t,mu(:,g),'Color',colors(k,:),'LineWidth',2.4);
    end
    xlabel('Time since selected movement start (s)');
    ylabel('Centered CH2 RMS (mV)'); grid on; box off;
    if panel==1, title('No weight'); else, title('15 lb'); end
    legend(handles,{'Baseline: no exoskeleton','Assisted: with exoskeleton'},'Location','best');
    xlim([0 t(end)]); set(ax,'FontSize',12);
end
linkaxes(axesList,'y');
title(layout,'Exploratory CH2 signal comparison','FontSize',19,'FontWeight','bold');
xlabel(layout,{'Lines: mean of 3 trials; shading: +/-1 SD across repeated trials.',...
    '51.2 Hz; 0.5 s RMS window; mean removed per selected interval; no frequency filter.',...
    'Drift and undersampling limit interpretation; lower RMS does not establish reduced muscle effort.'},'FontSize',10);
output = fullfile(folder,'EMG_comparison_output');
if ~exist(output,'dir'), mkdir(output); end
exportgraphics(fig,fullfile(output,'EMG_comparison.png'),'Resolution',300);
exportgraphics(fig,fullfile(output,'EMG_comparison.pdf'),'ContentType','vector');
savefig(fig,fullfile(output,'EMG_comparison.fig'));
curves = table(t,'VariableNames',{'Time_since_selected_start_s'});
for g=1:4
    curves.(char(groups(g)+'_Mean_RMS_mV')) = mu(:,g);
    curves.(char(groups(g)+'_SD_RMS_mV')) = sd(:,g);
end
writetable(curves,fullfile(output,'Average_curves.csv'));
selection = table(names(:),onset(:),finish(:),trialMean(:),...
    'VariableNames',{'Trial','Selected_start_s','Selected_end_s','Common_interval_mean_RMS_mV'});
writetable(selection,fullfile(output,'Selected_intervals.csv'));
disp(selection);
fprintf('Chart saved to %s\nEvery plotted point uses 3 trials.\n',output);
% Important: user-selected starts are alignment estimates, not detected onsets.
% If onset cannot be identified, use recording starts and describe that explicitly.
% Acquisition gaps are not repaired by sample-based time reconstruction.
% This chart is descriptive; no hypothesis test or causal benefit claim is made.
