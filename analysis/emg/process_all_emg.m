%% Process all 12 Shimmer EMG spreadsheets (CH2)
% HOW TO RUN:
% 1. Save this script in the folder containing the 12 original XLSX files.
% 2. Open the script in MATLAB and click Run.
% 3. Choose that data folder when prompted.
% Results are written to its emg_processed subfolder.
%
% Default mode requires ONLY base MATLAB. It removes the whole-record DC
% offset and calculates rolling RMS. It DOES NOT apply an EMG bandpass.
% The supplied exports imply ~51.2 Hz, too low for a 20-450 Hz EMG bandpass.
% RMS is exploratory signal amplitude, not validated muscle activation.
% Trials remain separate; no automatic alignment or averaging across trials.
clear; clc;

%% Settings
RMS_WINDOW_SECONDS = 0.100;
UNTIMED_WINDOW_SAMPLES = 5; % Explicit sample window if timing is unusable.
CONFIRMED_FS_HZ = []; % Leave empty unless acquisition rate is verified.
% With a verified rate, invalid timestamps can be replaced by sample time.
% The replacement is labeled reconstructed and is not the original clock.
ENABLE_BANDPASS = false; % Only enable for verified raw EMG at a suitable rate.
BANDPASS_HZ = [20 450];

inputFolder = uigetdir(pwd, 'Select the folder containing the 12 EMG XLSX files');
if isequal(inputFolder, 0), return; end
outputFolder = fullfile(inputFolder, 'emg_processed');
if ~exist(outputFolder, 'dir'), mkdir(outputFolder); end

if ~isempty(CONFIRMED_FS_HZ)
    validateattributes(CONFIRMED_FS_HZ, {'numeric'}, {'scalar','finite','positive'});
end
if ENABLE_BANDPASS && (isempty(which('butter')) || isempty(which('filtfilt')))
    error('Bandpass mode needs Signal Processing Toolbox. Keep ENABLE_BANDPASS=false for base MATLAB.');
end

conditions = {'BL_NW', 'BL_15', 'AS_NW', 'AS_15'};
files = dir(fullfile(inputFolder, '*.xlsx'));
summary = struct([]);
results = cell(4,3);
originalTimes = cell(4,3);
sharedClock = false(4,3);

%% Read the original columns without combining or padding trials
for c = 1:4
    for trial = 1:3
        prefix = sprintf('%s_%d_', conditions{c}, trial);
        matches = files(startsWith({files.name}, prefix));
        if isempty(matches)
            warning('Missing input for %s trial %d.', conditions{c}, trial);
            continue;
        elseif numel(matches) > 1
            error('More than one file matches %s. Remove duplicate copies from the selected folder.', prefix);
        end
        filename = matches(1).name;
        path = fullfile(inputFolder, filename);
        headers = readcell(path, 'Range', 'A1:C2');
        if size(headers,2) < 3 || ~contains(string(headers{1,3}), 'EMG_CH2')
            error('%s: expected CH2 in column C. Check the header before processing.', filename);
        end
        if ~strcmpi(strtrim(string(headers{2,1})), 'ms') || ...
                ~strcmpi(strtrim(string(headers{2,3})), 'mV')
            error('%s: expected timestamp units ms and CH2 units mV.', filename);
        end
        % Uploaded files have a header in row 1 and units in row 2.
        M = readmatrix(path, 'Range', 'A3');
        if size(M,2) < 3, error('%s: fewer than three columns.', filename); end
        originalTimes{c,trial} = M(:,1);
        xAll = M(:,3); % CH2; CH1 is not used.
        validSignal = find(isfinite(xAll));
        if numel(validSignal) < 10
            warning('%s: insufficient numeric CH2 readings; skipping.', filename);
            continue;
        end
        first = validSignal(1); last = validSignal(end);
        x = xAll(first:last);
        ts = M(first:last,1);
        sampleIndex = (0:numel(x)-1)';
        n = numel(x);
        notes = strings(0,1);
        if first > 1 || last < numel(xAll)
            notes(end+1) = "Removed only leading/trailing rows with no CH2 signal";
        end
        internalMissing = sum(~isfinite(x));
        if internalMissing > 0
            notes(end+1) = "Internal missing CH2 values preserved; affected RMS windows are blank";
        end

        dt = diff(ts)/1000;
        clockUsable = all(isfinite(ts)) && all(dt > 0);
        fsTimestamp = NaN;
        if clockUsable
            fsTimestamp = 1/median(dt);
            fs = fsTimestamp;
            timeSeconds = (ts-ts(1))/1000;
            timeBasis = "spreadsheet timestamps (not verified acquisition rate)";
            gapCount = sum(dt > 1.5*median(dt));
            if gapCount > 0
                notes(end+1) = "Timestamp gaps retained; signal was not interpolated";
            end
            if ~isempty(CONFIRMED_FS_HZ)
                fs = CONFIRMED_FS_HZ;
                if abs(fs-fsTimestamp)/fs > 0.02
                    notes(end+1) = "Verified rate disagrees with spreadsheet timing; timing needs review";
                end
            end
        elseif ~isempty(CONFIRMED_FS_HZ)
            fs = CONFIRMED_FS_HZ;
            timeSeconds = sampleIndex/fs;
            timeBasis = "reconstructed from verified sampling rate; assumes consecutive samples";
            gapCount = NaN;
            notes(end+1) = "Original timestamps invalid; time reconstructed from verified rate";
        else
            fs = NaN;
            timeSeconds = nan(n,1);
            timeBasis = "sample index only; original timestamps invalid";
            gapCount = NaN;
            notes(end+1) = "Original timestamps invalid; no seconds or acquisition rate invented";
        end
        if isfinite(fs)
            windowSamples = max(1, round(RMS_WINDOW_SECONDS*fs));
            actualWindowSeconds = windowSamples/fs;
            if fs < 1000
                notes(end+1) = "Low exported rate: RMS is exploratory; raw EMG bandwidth is unverified";
            end
        else
            windowSamples = UNTIMED_WINDOW_SAMPLES;
            actualWindowSeconds = NaN;
            notes(end+1) = "RMS uses an explicit 5-sample window, not a verified 100 ms window";
        end
        dc = mean(x(isfinite(x)));
        centered = x-dc;
        filtered = centered;
        method = "DC removal + rolling RMS; no bandpass";

        if ENABLE_BANDPASS
            if ~isfinite(fs) || BANDPASS_HZ(2) >= fs/2
                error('%s: bandpass requires high cutoff below Fs/2. Re-export data at a verified suitable rate.', filename);
            end
            if ~clockUsable || gapCount > 0 || internalMissing > 0
                error('%s: bandpass requires reviewed uniform continuous data with usable timestamps.', filename);
            end
            [b,a] = butter(4, BANDPASS_HZ/(fs/2), 'bandpass');
            filtered = filtfilt(b,a,centered);
            method = "DC removal + zero-phase bandpass + rolling RMS";
        end

        % Use full centered RMS windows; preserve missing values.
        before = floor((windowSamples-1)/2);
        after = windowSamples-1-before;
        rmsSignal = sqrt(movmean(filtered.^2, [before after]));
        badWindow = movsum(double(~isfinite(filtered)), [before after]) > 0;
        rmsSignal(badWindow) = NaN;
        if before > 0, rmsSignal(1:before) = NaN; end
        if after > 0, rmsSignal(end-after+1:end) = NaN; end
        % Do not calculate an RMS window across a timestamp gap.
        if clockUsable
            gaps = find(dt > 1.5*median(dt));
            for g = gaps'
                rmsSignal(max(1,g-after+1):min(n,g+before)) = NaN;
            end
        end
        goodRMS = rmsSignal(isfinite(rmsSignal));
        if isempty(goodRMS), wholeMean = NaN; else, wholeMean = mean(goodRMS); end
        label = sprintf('%s_%d', conditions{c},trial);
        T = table(sampleIndex, ts, timeSeconds, x, centered, filtered, rmsSignal, ...
            'VariableNames', {'SampleIndex','OriginalTimestamp_ms','Time_s', ...
            'CH2_raw_mV','CH2_centered_mV','CH2_processed_mV','CH2_rolling_RMS_mV'});
        writetable(T, fullfile(outputFolder, [label '_processed.csv']));
        results{c,trial} = struct('label',label,'table',T,'timeBasis',timeBasis,'windowSamples',windowSamples);
        row = struct('Condition',string(conditions{c}),'Trial',trial,'SourceFile',string(filename), ...
            'CH2Samples',n,'LeadingBlankRowsRemoved',first-1,'TrailingBlankRowsRemoved',numel(xAll)-last, ...
            'InternalMissingCH2',internalMissing,'TimestampImpliedFs_Hz',fsTimestamp, ...
            'RateUsedForWindow_Hz',fs,'TimeBasis',timeBasis,'LargeTimestampGaps',gapCount, ...
            'RMSWindowSamples',windowSamples,'RMSWindow_s',actualWindowSeconds, ...
            'DCOffset_mV',dc,'WholeRecordMeanRollingRMS_mV',wholeMean, ...
            'UsableRMSWindows',numel(goodRMS),'Method',method,'Notes',strjoin(notes,'; '));
        if isempty(summary), summary = row; else, summary(end+1) = row; end %#ok<SAGROW>
        fprintf('%s: %d CH2 samples; timestamp rate %.3f Hz; RMS window %d samples\n',label,n,fsTimestamp,windowSamples);
    end
end
if isempty(summary), error('No matching data processed. Select the original XLSX folder.'); end

%% Flag identical timestamp columns across distinct source files
for i = 1:numel(originalTimes)
    for j = i+1:numel(originalTimes)
        if ~isempty(originalTimes{i}) && ~isempty(originalTimes{j}) && ...
                isequaln(originalTimes{i},originalTimes{j})
            sharedClock(i) = true; sharedClock(j) = true;
        end
    end
end
for k = 1:numel(summary)
    c = find(strcmp(conditions, summary(k).Condition));
    trial = summary(k).Trial;
    if sharedClock(c,trial)
        summary(k).Notes = summary(k).Notes + "; Identical timestamp column in another input: verify original exports";
    end
end
S = struct2table(summary);
writetable(S, fullfile(outputFolder,'emg_trial_summary.csv'));
disp(S(:,{'Condition','Trial','CH2Samples','TimestampImpliedFs_Hz','WholeRecordMeanRollingRMS_mV'}));

%% Separate trials on consistent amplitude axes, without movement alignment
allRaw = []; allRMS = [];
for k = 1:numel(results)
    if isempty(results{k}), continue; end
    allRaw = [allRaw; results{k}.table.CH2_centered_mV]; %#ok<AGROW>
    allRMS = [allRMS; results{k}.table.CH2_rolling_RMS_mV]; %#ok<AGROW>
end
rawLimit = max(abs(allRaw(isfinite(allRaw))));
rmsLimit = max(allRMS(isfinite(allRMS)));
if isempty(rawLimit) || rawLimit == 0, rawLimit = 1; end
if isempty(rmsLimit) || rmsLimit == 0, rmsLimit = 1; end
for mode = 1:2
    fig = figure('Color','w','Position',[70 70 1250 900]);
    for c = 1:4
        for trial = 1:3
            ax = subplot(4,3,(c-1)*3+trial);
            r = results{c,trial};
            if isempty(r), title('Missing input'); axis off; continue; end
            T = r.table;
            if all(isfinite(T.Time_s))
                horizontal = T.Time_s;
                xlabel('Time (s; see timing notes)');
            else
                horizontal = T.SampleIndex;
                xlabel('Sample index (timing unavailable)');
            end
            if mode == 1
                plot(horizontal,T.CH2_centered_mV,'Color',[0.18 0.38 0.65]);
                ylim([-rawLimit rawLimit]*1.05); ylabel('DC-centered CH2 (mV)');
            else
                plot(horizontal,T.CH2_rolling_RMS_mV,'Color',[0.18 0.55 0.35],'LineWidth',1.2);
                ylim([0 rmsLimit*1.05]); ylabel('Rolling RMS (mV)');
            end
            title(sprintf('%s | %d-sample RMS',r.label,r.windowSamples),'Interpreter','none');
            grid on;
            if sharedClock(c,trial)
                text(ax,0.02,0.93,'Verify copied/shared timestamps','Units','normalized', ...
                    'FontSize',8,'Color',[0.7 0.2 0.15]);
            end
        end
    end
    if mode == 1
        sgtitle('CH2 recordings after DC removal | separate trials');
        name = 'emg_CH2_centered_all_trials';
    else
        sgtitle('Exploratory CH2 rolling RMS | no condition-effect claim');
        name = 'emg_CH2_RMS_all_trials';
    end
    print(fig,fullfile(outputFolder,[name '.png']),'-dpng','-r200');
    savefig(fig,fullfile(outputFolder,[name '.fig']));
end

%% Save a short method record alongside the results
fid = fopen(fullfile(outputFolder,'READ_ME.txt'),'w');
if fid ~= -1
    cleanupFile = onCleanup(@() fclose(fid));
    fprintf(fid,'CH2 only; source timestamp units ms and signal units mV.\n');
    fprintf(fid,'Original XLSX files were not modified. Only blank signal rows at the ends were trimmed.\n');
    fprintf(fid,'Whole-record DC offset removed; centered rolling RMS calculated. Bandpass enabled: %d.\n',ENABLE_BANDPASS);
    fprintf(fid,'Target RMS window %.3f s where a rate is available. Invalid clocks without a verified rate use %d samples and a sample-index axis.\n',RMS_WINDOW_SECONDS,UNTIMED_WINDOW_SAMPLES);
    fprintf(fid,'No trial alignment, row averaging, cross-trial averaging, significance tests, interpolation, or automatic curl detection.\n');
    fprintf(fid,'Summary means include all retained rest and movement. They are not matched lifting-phase measurements.\n');
    fprintf(fid,'Supplied usable timestamps imply about 51.2 Hz. This does not prove acquisition rate or signal bandwidth.\n');
    fprintf(fid,'Some timestamps are constant or identical across separate files. Check original Consensys exports/settings before relying on seconds.\n');
    fprintf(fid,'Low-rate exports cannot establish full-band raw EMG amplitude; digital processing cannot undo aliasing.\n');
    fprintf(fid,'Raw/envelope signal type, placement, gain, electrode contact, clipping, and movement artifacts remain unverified.\n');
    fprintf(fid,'Do not interpret these exploratory whole-record RMS differences as assistance benefit.\n');
    clear cleanupFile;
end
fprintf('\nSaved outputs to: %s\n',outputFolder);
fprintf('Before comparing muscle effort: verify exports/rate/signal type and select comparable movement phases.\n');
