% =========================================================================
% simulate_relay.m
% Simulation of differential_relay_v9.ino on recorded CSV signals
%
% Faithfully replicates firmware v9 logic:
%   Boot  : EFFECTIVE_RATIO = mean(I_pri_phases) / mean(I_sec_phases)
%           averaged over Healthy zone (mirrors runEffectiveRatioCalibration)
%   Loop  : per RMS-window (40 ms = 800 samples @20 kHz)
%             i_pri_comp = I_pri_rms / EFFECTIVE_RATIO
%             i_diff     = |i_pri_comp - I_sec_rms|
%             i_bias     = (i_pri_comp + I_sec_rms) / 2
%             trip       = shouldTrip(i_diff, i_bias)   [Dual Slope]
%
% Dataset assumptions
%   - CSV cols: EventNo,Date,Time,U1,U2,U3,U4,I1,I2,I3,I4
%   - I1 (col 8)  = Primary  line current  [A, already calibrated by Hioki]
%   - I4 (col 11) = Secondary line current [A]
%   - fs = 20 000 Hz,  f0 = 50 Hz
%   - 5 600 rows per file
%   - Transition Healthy→Fault located via sliding-RMS window on I1
%
% Outputs
%   Simulation_Output/simulation_results.csv  — per-file summary
%   Simulation_Output/simulation_log.csv      — per-window time series
% =========================================================================

clear; clc;

% =========================================================================
% 1.  FIRMWARE PARAMETERS  (direct copy from differential_relay_v9.ino)
% =========================================================================
FS             = 20000;   % Hz — Hioki PQ3198 recording rate
F0             = 50;      % Hz — supply fundamental
PROTECTION_MODE = 1;      % 1 = Dual Slope (IEC), 2 = Single Slope

% Dual Slope (PROTECTION_MODE == 1)
SLOPE1         = 0.10;
PICKUP         = 0.45;    % A  — MinPickup
KNEE_POINT     = 5.0;    % A  — knee Ibias
SLOPE2         = 0.50;
OFFSET2        = 1.55;    % A  — intercept correction for slope-2 line

% Single Slope (PROTECTION_MODE == 2)
SLOPE_S2       = 0.60;
PICKUP_S2      = 10.0;

% Calibration — ER averaged over how many healthy cycles
CAL_CYCLES     = 10;      % match TR_MEASURE_MS/cycle_len ≈ 500 ms / 1 cy
TR_MIN_CURRENT = 0.5;     % A — fallback trigger

% RMS window size (firmware: 40 ms accumulation @ 1 ms sampling)
% Hioki @ 20 kHz → 40 ms = 800 samples
SAMPLES_PER_CYCLE = FS / F0;           % 400
RMS_WIN           = round(0.040 * FS); % 800 samples  (= 2 cycles)

% =========================================================================
% 2.  shouldTrip() — exact translation of firmware function
% =========================================================================
shouldTrip = @(id, ib) tripLogic(id, ib, PROTECTION_MODE, ...
    SLOPE1, PICKUP, KNEE_POINT, SLOPE2, OFFSET2, SLOPE_S2, PICKUP_S2);

% =========================================================================
% 3.  TRANSITION DETECTION — sliding RMS window on I1 (win = 200 samples)
% =========================================================================
WIN_TRANS  = 200;   % samples — same as README step 4
BUFFER     = 300;   % samples — exclude transient around transition

% =========================================================================
% 4.  FIND FILES
% =========================================================================
base_dir = 'Selected_Signals';
if ~isfolder(base_dir)
    error('Folder "%s" not found. Run from the project root.', base_dir);
end

csv_files = dir(fullfile(base_dir, '**', '*.csv'));
n_files   = numel(csv_files);
if n_files == 0
    error('No CSV files found under %s', base_dir);
end
fprintf('Found %d CSV files.\n', n_files);

% =========================================================================
% 5.  OUTPUT SETUP
% =========================================================================
out_dir = 'Simulation_Output';
if ~isfolder(out_dir), mkdir(out_dir); end

fid_res = fopen(fullfile(out_dir, 'simulation_results.csv'), 'w');
fprintf(fid_res, ['File,Group,TransitionRow,ER,' ...
    'Healthy_FP_pct,Healthy_windows,' ...
    'Fault_Det_pct,Fault_windows,' ...
    'I_diff_healthy_mean,I_diff_fault_mean,' ...
    'PASS\n']);

fid_log = fopen(fullfile(out_dir, 'simulation_log.csv'), 'w');
fprintf(fid_log, ['File,WindowIdx,TimeSec,Zone,' ...
    'I_pri_rms,I_sec_rms,I_pri_comp,I_diff,I_bias,Threshold,Trip\n']);

% =========================================================================
% 6.  MAIN LOOP — one file at a time
% =========================================================================
for fi = 1:n_files
    fpath = fullfile(csv_files(fi).folder, csv_files(fi).name);
    fname = csv_files(fi).name;
    % derive group name from parent folder
    parts = strsplit(csv_files(fi).folder, filesep);
    group = parts{end};

    fprintf('[%3d/%d] %s / %s\n', fi, n_files, group, fname);

    % ------------------------------------------------------------------
    % 6a.  Load CSV
    % ------------------------------------------------------------------
    try
        raw = readmatrix(fpath, 'NumHeaderLines', 1);
    catch ME
        fprintf('  SKIP — cannot read file: %s\n', ME.message);
        continue
    end

    I1 = raw(:, 8);   % Primary  current [A]
    I4 = raw(:, 11);  % Secondary current [A]
    N  = length(I1);

    if N < WIN_TRANS * 2 + BUFFER * 2
        fprintf('  SKIP — too few samples (%d)\n', N);
        continue
    end

    % ------------------------------------------------------------------
    % 6b.  Find transition row (sliding RMS on I1, win=200)
    % ------------------------------------------------------------------
    trans_row = findTransition(I1, WIN_TRANS);

    % Zones (1-indexed)
    healthy_end = trans_row - BUFFER;
    fault_start  = trans_row + BUFFER;

    if healthy_end < WIN_TRANS + 1 || fault_start > N - RMS_WIN
        fprintf('  SKIP — transition too close to edge (row %d)\n', trans_row);
        continue
    end

    % ------------------------------------------------------------------
    % 6c.  EFFECTIVE_RATIO calibration — firmware boot step
    %       Mirror: measureRMSBlocking over CAL_CYCLES healthy cycles,
    %               then ER = mean(I_pri_rms_phases) / mean(I_sec_rms_phases)
    %       Here we only have Phase A primary (I1) and Phase A secondary (I4)
    %       → ER = RMS(I1_healthy_last_CAL) / RMS(I4_healthy_last_CAL)
    % ------------------------------------------------------------------
    cal_len   = CAL_CYCLES * SAMPLES_PER_CYCLE;
    cal_start = max(1, healthy_end - cal_len + 1);
    cal_end   = healthy_end;

    i_pri_cal = sqrt(mean(I1(cal_start:cal_end).^2));
    i_sec_cal = sqrt(mean(I4(cal_start:cal_end).^2));

    if i_pri_cal < TR_MIN_CURRENT || i_sec_cal < TR_MIN_CURRENT
        ER = 1.0;   % firmware fallback
        fprintf('  WARN — low calibration current (%.3fA / %.3fA) → ER=1.0\n', ...
            i_pri_cal, i_sec_cal);
    else
        ER = i_pri_cal / i_sec_cal;
    end
    fprintf('  trans_row=%d  ER=%.4f\n', trans_row, ER);

    % ------------------------------------------------------------------
    % 6d.  Per-window RMS + protection decision
    %       Firmware accumulates for 40 ms (800 smp) then evaluates.
    %       We step through the signal in non-overlapping 800-sample windows.
    % ------------------------------------------------------------------
    % Build window start indices for Healthy and Fault zones separately
    h_starts = (WIN_TRANS + 1) : RMS_WIN : (healthy_end - RMS_WIN + 1);
    f_starts = fault_start    : RMS_WIN : (N - RMS_WIN + 1);

    all_starts = [h_starts, f_starts];
    all_zones  = [repmat({'H'}, 1, numel(h_starts)), ...
                  repmat({'F'}, 1, numel(f_starts))];

    n_h_trip = 0;   n_h_win = numel(h_starts);
    n_f_trip = 0;   n_f_win = numel(f_starts);
    id_h_acc = [];
    id_f_acc = [];

    for wi = 1:numel(all_starts)
        ws = all_starts(wi);
        we = ws + RMS_WIN - 1;

        seg_pri = I1(ws:we);
        seg_sec = I4(ws:we);

        i_pri_rms  = sqrt(mean(seg_pri.^2));
        i_sec_rms  = sqrt(mean(seg_sec.^2));

        i_pri_comp = i_pri_rms / ER;
        i_diff_val = abs(i_pri_comp - i_sec_rms);
        i_bias_val = (i_pri_comp + i_sec_rms) / 2;

        trip = shouldTrip(i_diff_val, i_bias_val);
        thr  = computeThreshold(i_bias_val, PROTECTION_MODE, ...
                   SLOPE1, PICKUP, KNEE_POINT, SLOPE2, OFFSET2, ...
                   SLOPE_S2, PICKUP_S2);

        zone = all_zones{wi};
        t_sec = ws / FS;

        % Accumulate
        if strcmp(zone, 'H')
            if trip, n_h_trip = n_h_trip + 1; end
            id_h_acc(end+1) = i_diff_val; %#ok<AGROW>
        else
            if trip, n_f_trip = n_f_trip + 1; end
            id_f_acc(end+1) = i_diff_val; %#ok<AGROW>
        end

        % Write log row
        fprintf(fid_log, '%s,%s,%d,%.4f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n', ...
            fname, group, wi, t_sec, zone, ...
            i_pri_rms, i_sec_rms, i_pri_comp, i_diff_val, i_bias_val, thr, trip);
    end

    % ------------------------------------------------------------------
    % 6e.  Score
    % ------------------------------------------------------------------
    fp_pct  = (n_h_win > 0) * 100 * n_h_trip / max(n_h_win, 1);
    det_pct = (n_f_win > 0) * 100 * n_f_trip / max(n_f_win, 1);
    pass    = (n_h_trip == 0) && (n_f_trip == n_f_win) && (n_f_win > 0);

    id_h_mean = nanmean_safe(id_h_acc);
    id_f_mean = nanmean_safe(id_f_acc);

    fprintf('  H_windows=%d  FP=%.1f%%  |  F_windows=%d  Det=%.1f%%  |  %s\n', ...
        n_h_win, fp_pct, n_f_win, det_pct, ternary(pass,'PASS','FAIL'));

    % Write results row
    fprintf(fid_res, '%s,%s,%d,%.4f,%.2f,%d,%.2f,%d,%.4f,%.4f,%s\n', ...
        fname, group, trans_row, ER, ...
        fp_pct, n_h_win, det_pct, n_f_win, ...
        id_h_mean, id_f_mean, ternary(pass,'PASS','FAIL'));
end

fclose(fid_res);
fclose(fid_log);
fprintf('\nDone. Results in %s/\n', out_dir);

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function row = findTransition(I1, win)
% Returns the sample index with the maximum before/after RMS ratio.
N = length(I1);
best_row   = win + 1;
best_ratio = 0;
for i = win + 1 : N - win
    r_before = sqrt(mean(I1(i-win : i-1).^2));
    r_after  = sqrt(mean(I1(i     : i+win-1).^2));
    if r_before > 0
        r = r_after / r_before;
    else
        r = 0;
    end
    if r > best_ratio
        best_ratio = r;
        best_row   = i;
    end
end
row = best_row;
end

% --------------------------------------------------------------------------
function trip = tripLogic(id, ib, mode, ...
    slope1, pickup, knee, slope2, offset2, slope_s2, pickup_s2)
if mode == 1
    if ib < knee
        thr = slope1 * ib + pickup;
    else
        thr = slope2 * ib - offset2;
    end
    trip = id > thr;
else
    trip = id > slope_s2 * ib + pickup_s2;
end
end

% --------------------------------------------------------------------------
function thr = computeThreshold(ib, mode, ...
    slope1, pickup, knee, slope2, offset2, slope_s2, pickup_s2)
if mode == 1
    if ib < knee
        thr = slope1 * ib + pickup;
    else
        thr = slope2 * ib - offset2;
    end
else
    thr = slope_s2 * ib + pickup_s2;
end
end

% --------------------------------------------------------------------------
function v = nanmean_safe(arr)
if isempty(arr)
    v = NaN;
else
    v = mean(arr(~isnan(arr)));
end
end

% --------------------------------------------------------------------------
function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end

