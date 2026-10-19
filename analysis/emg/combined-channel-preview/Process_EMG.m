% Put this script beside EMG_ALL.csv. Click Run.
clear; clc; close all;
folder = fileparts(mfilename('fullpath'));
T = readtable(fullfile(folder,'EMG_ALL.csv'),'VariableNamingRule','preserve');
labels = string(T.Properties.VariableNames(2:end));
% Confirm this in Consensys. Intact CSV timestamps suggest approximately 51.2 Hz.
Fs = []; % Example AFTER confirmation: Fs = 51.2;
windowSamples = 26; % Used only for sample-based RMS preview when Fs is unknown.
windowSeconds = 0.5;
R = nan(height(T),numel(labels));
means = nan(numel(labels),1); counts = zeros(numel(labels),1);
for k = 1:numel(labels)
    x = T{:,k+1};
    last = find(isfinite(x),1,'last');
    if isempty(last), continue; end
    x = x(1:last);
    if any(~isfinite(x))
        warning('Skipping %s: missing values inside recording.',labels(k));
        continue;
    end
    x = x - mean(x);
    if ~isempty(Fs)
        if Fs <= 40
            error('Sampling rate too low for the configured 20 Hz high-pass edge.');
        end
        high = min(450,0.9*(Fs/2));
        [b,a] = butter(4,[20 high]/(Fs/2),'bandpass');
        x = filtfilt(b,a,x);
        win = max(1,round(windowSeconds*Fs));
    else
        win = windowSamples;
    end
    y = sqrt(movmean(x.^2,win));
    R(1:last,k) = y;
    counts(k) = last;
    % Whole-recording preview: select matched activity intervals before comparisons.
    means(k) = mean(y);
end
if isempty(Fs)
    axisValues = T.Sample;
    axisLabel = 'Sample number';
    status = 'DC-centered RMS preview; no frequency filter';
else
    axisValues = (T.Sample-1)/Fs;
    axisLabel = 'Elapsed time (s; assumes uniform acquisition)';
    status = sprintf('Bandpass RMS; Fs = %.4g Hz',Fs);
end
results = array2table(R,'VariableNames',cellstr(labels));
results = addvars(results,axisValues,'Before',1,'NewVariableNames','Sample_or_time');
writetable(results,fullfile(folder,'RMS_results.csv'));
summary = table(labels(:),counts,means,'VariableNames',{'Run_Channel','Samples','Whole_recording_mean_RMS_mV'});
writetable(summary,fullfile(folder,'RMS_preview_summary.csv'));
figure; tiledlayout(2,1);
for channel = 1:2
    nexttile;
    columns = channel:2:numel(labels);
    plot(axisValues,R(:,columns));
    xlabel(axisLabel); ylabel('RMS (mV)');
    title(sprintf('CH%d: %s',channel,status));
    legend(labels(columns),'Interpreter','none','Location','eastoutside'); grid on;
end
fprintf('%s\nOutputs saved beside this script.\n',status);
