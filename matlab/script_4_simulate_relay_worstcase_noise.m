% =========================================================================
% simulate_relay_worstcase_noise.m
% Worst-Case Noise Simulation — based on simulate_relay.m
%
% add noise injection from hardware noise budget (README Section 5):
%
%   I_diff_noise(I_bias) = ADC_CONTRIBUTION + CT_SLOPE × I_bias
%     ADC_CONTRIBUTION  = 2 LSB × CT_RATIO × R_BURDEN = 0.0160 A
%     CT_SLOPE          = 2 × (CT_ratio_err + CT_phase_err) ≈ 0.026
%   → worst-case I_diff_noise ≈ 0.016 + 0.026 × I_bias  [A]
%
% Noise injection strategy (additive, worst direction):
%   I_pri_noisy = seg_pri × (1 + CT_ERROR_FRAC) + ADC_OFFSET_A  [+DC +gain]
%   I_sec_noisy = seg_sec × (1 - CT_ERROR_FRAC) - ADC_OFFSET_A  [-DC -gain]
%   → I_diff_noisy increases by ≈ ADC_CONTRIBUTION + CT_SLOPE × I_bias
%
% Outputs (all in Simulation_Output/):
%   simulation_log_noisy.csv     — per-window: clean vs noisy I_diff + margin
%   simulation_results_noisy.csv — per-file: PASS/FAIL clean vs noisy
%   + console summary: worst-margin files, noise budget stats
%
% Using same Parameters & firmware logic with simulate_relay.m
% Only Section 2 (NOISE BUDGET PARAMETERS) can be modify
% =========================================================================

clear; clc;

% =========================================================================
% 1.  FIRMWARE PARAMETERS  (sync กับ differential_relay_v9.ino)
% =========================================================================
FS              = 20000;
F0              = 50;
PROTECTION_MODE = 1;

% Dual Slope (PROTECTION_MODE == 1)
SLOPE1         = 0.10;
PICKUP         = 0.45;    % A  — MinPickup
KNEE_POINT     = 5.0;    % A  — knee Ibias
SLOPE2         = 0.50;
OFFSET2        = 1.55;    % A  — intercept correction for slope-2 line

% Single Slope
SLOPE_S2        = 0.60;
PICKUP_S2       = 10.0;

% Calibration
CAL_CYCLES      = 10;
TR_MIN_CURRENT  = 0.5;

% RMS window
SAMPLES_PER_CYCLE = FS / F0;            % 400
RMS_WIN           = round(0.040 * FS);  % 800 samples

% =========================================================================
% 2.  NOISE BUDGET PARAMETERS  (CHANGE HERE ONLY)
% =========================================================================
% -- ADC noise (systematic DC offset, worst-case 2 LSB) --
ADC_LSB_V       = 3.3 / 4096;      % ≈ 0.000806 V per LSB
ADC_OFFSET_LSB  = 2;                % worst-case systematic offset (LSB)
CT_RATIO_HW     = 60.0;             % A/V  (30A/5A ÷ 0.10 Ω)
ADC_OFFSET_A    = ADC_OFFSET_LSB * ADC_LSB_V * CT_RATIO_HW;
% → 2 × 0.000806 × 60 ≈ 0.0097 A  (per channel, opposite direction)

% -- CT Class 0.5 error --
CT_RATIO_ERR    = 0.005;            % ±0.5% ratio error per CT
CT_PHASE_ERR_DEG= 0.5 / 60;        % 30 arcmin → degrees  (≈ 0.00833°)
CT_PHASE_ERR_RAD= CT_PHASE_ERR_DEG * pi / 180;
% Combined worst-case fraction: ratio + phase (orthogonal approx)
CT_ERROR_FRAC   = sqrt(CT_RATIO_ERR^2 + sin(CT_PHASE_ERR_RAD)^2);
% ≈ 0.0130  (1.30%)

% Analytical noise floor for console report
noise_fn = @(ib) 2*ADC_OFFSET_A + 2*CT_ERROR_FRAC*ib;
% (×2 because Pri and Sec errors add in worst direction)

fprintf('=== WORST-CASE NOISE PARAMETERS ===\n');
fprintf('ADC offset per channel : %.4f A  (%d LSB)\n', ADC_OFFSET_A, ADC_OFFSET_LSB);
fprintf('CT error fraction      : %.4f  (%.2f%%)\n', CT_ERROR_FRAC, CT_ERROR_FRAC*100);
fprintf('Noise floor formula    : %.4f + %.4f × I_bias  [A]\n', 2*ADC_OFFSET_A, 2*CT_ERROR_FRAC);
fprintf('At I_bias=3.2 A        : %.4f A  (README: ~0.099 A)\n', noise_fn(3.2));
fprintf('====================================\n\n');

% =========================================================================
% 3.  shouldTrip + computeThreshold helpers
% =========================================================================
shouldTrip = @(id, ib) tripLogic(id, ib, PROTECTION_MODE, ...
    SLOPE1, PICKUP, KNEE_POINT, SLOPE2, OFFSET2, SLOPE_S2, PICKUP_S2);

% =========================================================================
% 4.  TRANSITION DETECTION PARAMETERS
% =========================================================================
WIN_TRANS = 200;
BUFFER    = 300;

% =========================================================================
% 5.  FIND FILES
% =========================================================================
base_dir = 'Selected_Signals';
if ~isfolder(base_dir)
    error('Folder "%s" not found. Run from project root.', base_dir);
end
csv_files = dir(fullfile(base_dir, '**', '*.csv'));
n_files   = numel(csv_files);
if n_files == 0, error('No CSV files found under %s', base_dir); end
fprintf('Found %d CSV files.\n\n', n_files);

% =========================================================================
% 6.  OUTPUT SETUP
% =========================================================================
out_dir = 'Simulation_Output';
if ~isfolder(out_dir), mkdir(out_dir); end

% -- Log file (per-window) --
fid_log = fopen(fullfile(out_dir, 'simulation_log_noisy.csv'), 'w');
fprintf(fid_log, ['File,Group,WindowIdx,TimeSec,Zone,' ...
    'I_pri_rms,I_sec_rms,' ...
    'I_diff_clean,I_diff_noisy,' ...
    'I_bias_clean,I_bias_noisy,' ...
    'Threshold,' ...
    'Noise_budget,' ...
    'Margin_clean,Margin_noisy,' ...
    'Trip_clean,Trip_noisy\n']);

% -- Results file (per-file) --
fid_res = fopen(fullfile(out_dir, 'simulation_results_noisy.csv'), 'w');
fprintf(fid_res, ['File,Group,TransitionRow,ER,' ...
    'H_windows,FP_clean,FP_noisy,' ...
    'F_windows,Det_clean,Det_noisy,' ...
    'MinMargin_H_clean,MinMargin_H_noisy,' ...
    'MinMargin_F_clean,MinMargin_F_noisy,' ...
    'PASS_clean,PASS_noisy\n']);

% =========================================================================
% 7.  MAIN LOOP
% =========================================================================
% Accumulators for final console summary
summary = struct();
summary_idx = 0;

for fi = 1:n_files
    fpath = fullfile(csv_files(fi).folder, csv_files(fi).name);
    fname = csv_files(fi).name;
    parts = strsplit(csv_files(fi).folder, filesep);
    group = parts{end};

    fprintf('[%3d/%d] %s / %s\n', fi, n_files, group, fname);

    % ------------------------------------------------------------------
    % 7a.  Load CSV
    % ------------------------------------------------------------------
    try
        raw = readmatrix(fpath, 'NumHeaderLines', 1);
    catch ME
        fprintf('  SKIP — %s\n', ME.message);
        continue
    end
    I1 = raw(:, 8);
    I4 = raw(:, 11);
    N  = length(I1);
    if N < WIN_TRANS*2 + BUFFER*2
        fprintf('  SKIP — too few samples\n');
        continue
    end

    % ------------------------------------------------------------------
    % 7b.  Find transition
    % ------------------------------------------------------------------
    trans_row   = findTransition(I1, WIN_TRANS);
    healthy_end = trans_row - BUFFER;
    fault_start = trans_row + BUFFER;
    if healthy_end < WIN_TRANS+1 || fault_start > N-RMS_WIN
        fprintf('  SKIP — transition too close to edge\n');
        continue
    end

    % ------------------------------------------------------------------
    % 7c.  EFFECTIVE_RATIO calibration (clean signal — mirrors firmware boot)
    % ------------------------------------------------------------------
    cal_len   = CAL_CYCLES * SAMPLES_PER_CYCLE;
    cal_start = max(1, healthy_end - cal_len + 1);
    i_pri_cal = sqrt(mean(I1(cal_start:healthy_end).^2));
    i_sec_cal = sqrt(mean(I4(cal_start:healthy_end).^2));
    if i_pri_cal < TR_MIN_CURRENT || i_sec_cal < TR_MIN_CURRENT
        ER = 1.0;
    else
        ER = i_pri_cal / i_sec_cal;
    end
    fprintf('  trans_row=%d  ER=%.4f\n', trans_row, ER);

    % ------------------------------------------------------------------
    % 7d.  Window indices
    % ------------------------------------------------------------------
    h_starts  = (WIN_TRANS+1) : RMS_WIN : (healthy_end - RMS_WIN + 1);
    f_starts  = fault_start   : RMS_WIN : (N - RMS_WIN + 1);
    all_starts = [h_starts, f_starts];
    all_zones  = [repmat({'H'}, 1, numel(h_starts)), ...
                  repmat({'F'}, 1, numel(f_starts))];

    % Counters
    n_h_win = numel(h_starts);
    n_f_win = numel(f_starts);
    n_h_trip_clean = 0;  n_h_trip_noisy = 0;
    n_f_trip_clean = 0;  n_f_trip_noisy = 0;

    % Margin tracking
    min_margin_H_clean =  Inf;  min_margin_H_noisy =  Inf;
    min_margin_F_clean =  Inf;  min_margin_F_noisy =  Inf;

    % ------------------------------------------------------------------
    % 7e.  Per-window loop
    % ------------------------------------------------------------------
    for wi = 1:numel(all_starts)
        ws = all_starts(wi);
        we = ws + RMS_WIN - 1;

        seg_pri = I1(ws:we);
        seg_sec = I4(ws:we);

        % === CLEAN ===
        i_pri_rms_c  = sqrt(mean(seg_pri.^2));
        i_sec_rms_c  = sqrt(mean(seg_sec.^2));
        i_pri_comp_c = i_pri_rms_c / ER;
        i_diff_c     = abs(i_pri_comp_c - i_sec_rms_c);
        i_bias_c     = (i_pri_comp_c + i_sec_rms_c) / 2;
        thr_c        = computeThreshold(i_bias_c, PROTECTION_MODE, ...
                           SLOPE1, PICKUP, KNEE_POINT, SLOPE2, OFFSET2, ...
                           SLOPE_S2, PICKUP_S2);
        trip_c       = shouldTrip(i_diff_c, i_bias_c);

        % === NOISY — worst-case injection ===
        % Pri: +CT_gain error, +ADC offset  → RMS increases
        % Sec: -CT_gain error, -ADC offset  → RMS decreases
        % Result: i_diff_noisy > i_diff_clean (worst direction)
        seg_pri_n = seg_pri * (1 + CT_ERROR_FRAC) + ADC_OFFSET_A;
        seg_sec_n = seg_sec * (1 - CT_ERROR_FRAC) - ADC_OFFSET_A;

        i_pri_rms_n  = sqrt(mean(seg_pri_n.^2));
        i_sec_rms_n  = sqrt(mean(seg_sec_n.^2));
        i_pri_comp_n = i_pri_rms_n / ER;
        i_diff_n     = abs(i_pri_comp_n - i_sec_rms_n);
        i_bias_n     = (i_pri_comp_n + i_sec_rms_n) / 2;
        thr_n        = computeThreshold(i_bias_n, PROTECTION_MODE, ...
                           SLOPE1, PICKUP, KNEE_POINT, SLOPE2, OFFSET2, ...
                           SLOPE_S2, PICKUP_S2);
        trip_n       = shouldTrip(i_diff_n, i_bias_n);

        % Noise budget at this operating point
        noise_budget = noise_fn(i_bias_c);

        % Margins  (positive = safe gap from threshold)
        %   Healthy zone: want I_diff < threshold → margin = threshold - I_diff
        %   Fault zone  : want I_diff > threshold → margin = I_diff - threshold
        zone = all_zones{wi};
        if strcmp(zone, 'H')
            margin_c = thr_c - i_diff_c;
            margin_n = thr_n - i_diff_n;
            if trip_c, n_h_trip_clean = n_h_trip_clean + 1; end
            if trip_n, n_h_trip_noisy = n_h_trip_noisy + 1; end
            min_margin_H_clean = min(min_margin_H_clean, margin_c);
            min_margin_H_noisy = min(min_margin_H_noisy, margin_n);
        else
            margin_c = i_diff_c - thr_c;
            margin_n = i_diff_n - thr_n;
            if trip_c, n_f_trip_clean = n_f_trip_clean + 1; end
            if trip_n, n_f_trip_noisy = n_f_trip_noisy + 1; end
            min_margin_F_clean = min(min_margin_F_clean, margin_c);
            min_margin_F_noisy = min(min_margin_F_noisy, margin_n);
        end

        % Write log row
        fprintf(fid_log, '%s,%s,%d,%.4f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d\n', ...
            fname, group, wi, ws/FS, zone, ...
            i_pri_rms_c, i_sec_rms_c, ...
            i_diff_c, i_diff_n, ...
            i_bias_c, i_bias_n, ...
            thr_c, noise_budget, ...
            margin_c, margin_n, ...
            trip_c, trip_n);
    end

    % ------------------------------------------------------------------
    % 7f.  Score
    % ------------------------------------------------------------------
    fp_clean  = safe_pct(n_h_trip_clean, n_h_win);
    fp_noisy  = safe_pct(n_h_trip_noisy, n_h_win);
    det_clean = safe_pct(n_f_trip_clean, n_f_win);
    det_noisy = safe_pct(n_f_trip_noisy, n_f_win);

    pass_clean = (n_h_trip_clean == 0) && (n_f_trip_clean == n_f_win) && (n_f_win > 0);
    pass_noisy = (n_h_trip_noisy == 0) && (n_f_trip_noisy == n_f_win) && (n_f_win > 0);

    % Guard Inf (no windows in zone)
    if isinf(min_margin_H_clean), min_margin_H_clean = NaN; end
    if isinf(min_margin_H_noisy), min_margin_H_noisy = NaN; end
    if isinf(min_margin_F_clean), min_margin_F_clean = NaN; end
    if isinf(min_margin_F_noisy), min_margin_F_noisy = NaN; end

    status_str = '';
    if pass_clean && pass_noisy
        status_str = 'PASS/PASS';
    elseif pass_clean && ~pass_noisy
        status_str = 'PASS/FAIL ← noise exposed';
    elseif ~pass_clean
        status_str = 'FAIL/FAIL';
    end

    fprintf('  H=%d FP:%.0f%%/%.0f%% | F=%d Det:%.0f%%/%.0f%% | Margin_H:%.3f/%.3f | Margin_F:%.3f/%.3f | %s\n', ...
        n_h_win, fp_clean, fp_noisy, ...
        n_f_win, det_clean, det_noisy, ...
        nanval(min_margin_H_clean), nanval(min_margin_H_noisy), ...
        nanval(min_margin_F_clean), nanval(min_margin_F_noisy), ...
        status_str);

    % Write results row
    fprintf(fid_res, '%s,%s,%d,%.4f,%d,%.2f,%.2f,%d,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%s,%s\n', ...
        fname, group, trans_row, ER, ...
        n_h_win, fp_clean, fp_noisy, ...
        n_f_win, det_clean, det_noisy, ...
        nanval(min_margin_H_clean), nanval(min_margin_H_noisy), ...
        nanval(min_margin_F_clean), nanval(min_margin_F_noisy), ...
        ternary(pass_clean,'PASS','FAIL'), ternary(pass_noisy,'PASS','FAIL'));

    % Save for summary
    summary_idx = summary_idx + 1;
    summary(summary_idx).fname         = fname;
    summary(summary_idx).group         = group;
    summary(summary_idx).pass_clean    = pass_clean;
    summary(summary_idx).pass_noisy    = pass_noisy;
    summary(summary_idx).margin_H_n    = nanval(min_margin_H_noisy);
    summary(summary_idx).margin_F_n    = nanval(min_margin_F_noisy);
    summary(summary_idx).margin_H_c    = nanval(min_margin_H_clean);
    summary(summary_idx).margin_F_c    = nanval(min_margin_F_clean);
end

fclose(fid_log);
fclose(fid_res);

% =========================================================================
% 8.  CONSOLE SUMMARY
% =========================================================================
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║               WORST-CASE NOISE SIMULATION SUMMARY            ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

n_total      = summary_idx;
n_pass_clean = sum([summary.pass_clean]);
n_pass_noisy = sum([summary.pass_noisy]);
n_exposed    = sum([summary.pass_clean] & ~[summary.pass_noisy]);

fprintf('║  Files processed     : %3d                                   ║\n', n_total);
fprintf('║  PASS (clean)        : %3d / %d                              ║\n', n_pass_clean, n_total);
fprintf('║  PASS (noisy)        : %3d / %d                              ║\n', n_pass_noisy, n_total);
fprintf('║  Noise exposed FAIL  : %3d  (pass clean → fail noisy)        ║\n', n_exposed);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

% Worst Healthy margin (noisy) — smallest positive value = most at risk of FP
all_mH = [summary.margin_H_n];
[mH_min, mH_idx] = min(all_mH);
fprintf('║  Worst Healthy margin (noisy)                                ║\n');
fprintf('║    %.4f A  →  %s / %s\n', mH_min, summary(mH_idx).group, summary(mH_idx).fname);

% Worst Fault margin (noisy) — smallest value = weakest detection
all_mF = [summary.margin_F_n];
[mF_min, mF_idx] = min(all_mF);
fprintf('║  Worst Fault margin (noisy)                                  ║\n');
fprintf('║    %.4f A  →  %s / %s\n', mF_min, summary(mF_idx).group, summary(mF_idx).fname);

fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  NOISE BUDGET at worst operating points:                     ║\n');
fprintf('║    ADC offset  : %.4f A per channel (×2 = %.4f A)            ║\n', ADC_OFFSET_A, 2*ADC_OFFSET_A);
fprintf('║    CT error    : %.4f frac → at I_bias=3.2A: %.4f A          ║\n', CT_ERROR_FRAC, 2*CT_ERROR_FRAC*3.2);
fprintf('║    Total worst : 0.016 + 0.026 × I_bias  (README Sec.5)      ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

if n_exposed > 0
    fprintf('║  ⚠  FILES WHERE NOISE CAUSES FAILURE:                       ║\n');
    for k = 1:n_total
        if summary(k).pass_clean && ~summary(k).pass_noisy
            fprintf('║    %s / %s\n', summary(k).group, summary(k).fname);
        end
    end
else
    fprintf('║  ✓  No files flipped by worst-case noise — margins adequate  ║\n');
end
fprintf('╚══════════════════════════════════════════════════════════════╝\n');
fprintf('\nDone. Results in %s/\n', out_dir);

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function row = findTransition(I1, win)
N = length(I1);
best_row = win + 1;  best_ratio = 0;
for i = win+1 : N-win
    rb = sqrt(mean(I1(i-win : i-1).^2));
    ra = sqrt(mean(I1(i     : i+win-1).^2));
    r  = (rb > 0) * ra / max(rb, 1e-9);
    if r > best_ratio, best_ratio = r; best_row = i; end
end
row = best_row;
end

function trip = tripLogic(id, ib, mode, s1, pu, knee, s2, off2, ss2, pus2)
if mode == 1
    thr = (ib < knee) * (s1*ib + pu) + (ib >= knee) * (s2*ib - off2);
    trip = id > thr;
else
    trip = id > ss2*ib + pus2;
end
end

function thr = computeThreshold(ib, mode, s1, pu, knee, s2, off2, ss2, pus2)
if mode == 1
    if ib < knee, thr = s1*ib + pu; else, thr = s2*ib - off2; end
else
    thr = ss2*ib + pus2;
end
end

function v = safe_pct(num, den)
if den == 0, v = NaN; else, v = 100*num/den; end
end

function v = nanval(x)
if isnan(x), v = -999; else, v = x; end
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end

