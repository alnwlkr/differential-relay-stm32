% =========================================================================
% script_2_bias_characteristic.m
% Differential Bias Characteristic Analysis
%
% Goal:
%   1. Calcuate I_diff and I_bias of every window using every files in Selected_Signals/
%   2. Plot Scatter (Healthy vs Fault) บน I_bias–I_diff plane
%      พร้อม Dual-Slope characteristic curve ให้ปรับ tune ด้วยตา
%   3. Grid Search หาชุด PICKUP, SLOPE1, SLOPE2, KNEE_POINT
%      ที่ให้ FP=0% และ Detection=100% บน dataset ทั้งหมด
%      แล้ว report ออกมาเป็น CSV เรียงตาม margin ดีสุด
%
% Input  : Selected_Signals/**/*.csv
% Output :
%   Simulation_Output/bias_scatter.png          — scatter plot รวมทุกไฟล์
%   Simulation_Output/bias_characteristic.csv   — raw I_diff/I_bias ทุก window
%   Simulation_Output/grid_search_results.csv   — ผล grid search เรียงตาม margin
%   Simulation_Output/grid_search_best.txt      — ค่าที่ดีที่สุด (พร้อม copy ไป firmware)
%
% *** TUNING PARAMETERS in Section 1 to change curve of scatter plot ***
% *** Grid Search range can be modified in  Section 2 ***
% =========================================================================

clear; clc;

% =========================================================================
% 1.  PARAMETERS that used to draw characteristic curve of scatter plot
%     (modify these value for visual tuning)
% =========================================================================
FS              = 20000;   % Hz
F0              = 50;      % Hz
RMS_WIN         = round(0.040 * FS);   % 800 samples (40 ms)
SAMPLES_PER_CYCLE = FS / F0;           % 400

% Transition detection
WIN_TRANS       = 200;
BUFFER          = 300;
CAL_CYCLES      = 10;
TR_MIN_CURRENT  = 0.5;

% --- Characteristic curve parameters  ---
PICKUP_PLOT     = 0.45;   % A   — MinPickup (intercept ของ Slope 1 ที่ Ib=0)
SLOPE1_PLOT     = 0.2;   % —   — Slope ช่วง low current
SLOPE2_PLOT     = 0.60;   % —   — Slope ช่วง high current
KNEE_PLOT       = 5.0;   % A   — จุดเปลี่ยน slope (knee point)
% calculate OFFSET2 for knee point (continuity):
%   slope1*knee + pickup = slope2*knee - offset2
%   offset2 = (slope2 - slope1)*knee - pickup
OFFSET2_PLOT    = (SLOPE2_PLOT - SLOPE1_PLOT) * KNEE_PLOT - PICKUP_PLOT;

fprintf('=== Characteristic Curve Parameters ===\n');
fprintf('PICKUP    = %.3f A\n', PICKUP_PLOT);
fprintf('SLOPE1    = %.3f\n',   SLOPE1_PLOT);
fprintf('SLOPE2    = %.3f\n',   SLOPE2_PLOT);
fprintf('KNEE      = %.3f A\n', KNEE_PLOT);
fprintf('OFFSET2   = %.3f A  (auto-calculated for continuity)\n', OFFSET2_PLOT);
fprintf('  [Verify: at Ib=%.1f → thr1=%.3f, thr2=%.3f (should match)]\n', ...
    KNEE_PLOT, SLOPE1_PLOT*KNEE_PLOT+PICKUP_PLOT, SLOPE2_PLOT*KNEE_PLOT-OFFSET2_PLOT);
fprintf('========================================\n\n');

% =========================================================================
% 2.  GRID SEARCH PARAMETERS 
% =========================================================================
gs_pickup  = 0.20 : 0.05 : 1.00;    % A    — MinPickup
gs_slope1  = 0.10 : 0.05 : 0.50;    % —    — Slope 1
gs_slope2  = 0.50 : 0.10 : 1.20;    % —    — Slope 2
gs_knee    = [5, 8, 10, 12, 15];    % A    — Knee point

fprintf('Grid Search: %d×%d×%d×%d = %d combinations\n', ...
    numel(gs_pickup), numel(gs_slope1), numel(gs_slope2), numel(gs_knee), ...
    numel(gs_pickup)*numel(gs_slope1)*numel(gs_slope2)*numel(gs_knee));
fprintf('(This may take a moment...)\n\n');

% =========================================================================
% 3.  FIND FILES
% =========================================================================
base_dir = 'Selected_Signals';
if ~isfolder(base_dir)
    error('Folder "%s" not found. Run from project root.', base_dir);
end
csv_files = dir(fullfile(base_dir, '**', '*.csv'));
n_files   = numel(csv_files);
if n_files == 0
    error('No CSV files found under %s', base_dir);
end
fprintf('Found %d CSV files.\n\n', n_files);

% =========================================================================
% 4.  OUTPUT SETUP
% =========================================================================
out_dir = 'Simulation_Output';
if ~isfolder(out_dir), mkdir(out_dir); end

% =========================================================================
% 5.  EXTRACT I_diff / I_bias FROM ALL FILES
% =========================================================================
% Pre-allocate accumulators
all_ib_h = [];   all_id_h = [];   % Healthy zone
all_ib_f = [];   all_id_f = [];   % Fault zone

fid_raw = fopen(fullfile(out_dir, 'bias_characteristic.csv'), 'w');
fprintf(fid_raw, 'File,Group,WindowIdx,Zone,I_pri_rms,I_sec_rms,ER,I_pri_comp,I_diff,I_bias\n');

fprintf('--- Pass 1: Extracting I_diff / I_bias ---\n');

for fi = 1:n_files
    fpath = fullfile(csv_files(fi).folder, csv_files(fi).name);
    fname = csv_files(fi).name;
    parts = strsplit(csv_files(fi).folder, filesep);
    group = parts{end};

    fprintf('[%3d/%d] %s / %s\n', fi, n_files, group, fname);

    try
        raw = readmatrix(fpath, 'NumHeaderLines', 1);
    catch ME
        fprintf('  SKIP — %s\n', ME.message); continue
    end

    I1 = raw(:,8);  I4 = raw(:,11);  N = length(I1);
    if N < WIN_TRANS*2 + BUFFER*2
        fprintf('  SKIP — too few samples\n'); continue
    end

    % --- Transition ---
    trans_row   = findTransition(I1, WIN_TRANS);
    healthy_end = trans_row - BUFFER;
    fault_start = trans_row + BUFFER;
    if healthy_end < WIN_TRANS+1 || fault_start > N-RMS_WIN
        fprintf('  SKIP — transition at edge\n'); continue
    end

    % --- EFFECTIVE_RATIO ---
    cal_len   = CAL_CYCLES * SAMPLES_PER_CYCLE;
    cal_start = max(1, healthy_end - cal_len + 1);
    rms_p_cal = sqrt(mean(I1(cal_start:healthy_end).^2));
    rms_s_cal = sqrt(mean(I4(cal_start:healthy_end).^2));
    if rms_p_cal < TR_MIN_CURRENT || rms_s_cal < TR_MIN_CURRENT
        ER = 1.0;
    else
        ER = rms_p_cal / rms_s_cal;
    end

    % --- Windows ---
    h_starts = (WIN_TRANS+1) : RMS_WIN : (healthy_end - RMS_WIN + 1);
    f_starts = fault_start   : RMS_WIN : (N - RMS_WIN + 1);
    all_starts = [h_starts, f_starts];
    all_zones  = [repmat({'H'},1,numel(h_starts)), repmat({'F'},1,numel(f_starts))];

    for wi = 1:numel(all_starts)
        ws = all_starts(wi);
        we = ws + RMS_WIN - 1;

        rms_p  = sqrt(mean(I1(ws:we).^2));
        rms_s  = sqrt(mean(I4(ws:we).^2));
        p_comp = rms_p / ER;
        i_diff = abs(p_comp - rms_s);
        i_bias = (p_comp + rms_s) / 2;
        zone   = all_zones{wi};

        if strcmp(zone, 'H')
            all_ib_h(end+1) = i_bias;  %#ok<AGROW>
            all_id_h(end+1) = i_diff;  %#ok<AGROW>
        else
            all_ib_f(end+1) = i_bias;  %#ok<AGROW>
            all_id_f(end+1) = i_diff;  %#ok<AGROW>
        end

        fprintf(fid_raw, '%s,%s,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', ...
            fname, group, wi, zone, rms_p, rms_s, ER, p_comp, i_diff, i_bias);
    end
end
fclose(fid_raw);

fprintf('\nTotal windows — Healthy: %d   Fault: %d\n\n', numel(all_ib_h), numel(all_ib_f));

% =========================================================================
% 6.  SCATTER PLOT
% =========================================================================
fprintf('--- Generating scatter plot ---\n');

ib_range = [0, max([all_ib_h, all_ib_f]) * 1.05];
ib_curve = linspace(0, ib_range(2), 500);

% Characteristic curve from TUNING PARAMETERS
id_curve = arrayfun(@(ib) dualSlopeThr(ib, SLOPE1_PLOT, PICKUP_PLOT, KNEE_PLOT, ...
                                            SLOPE2_PLOT, OFFSET2_PLOT), ib_curve);

fig = figure('Visible','off','Position',[0 0 1100 650]);
hold on; grid on; box on;

% Scatter: Healthy (blue circles) and Fault (red crosses)

scatter(all_ib_h, all_id_h, 20, [0.2 0.4 0.8], 'o', 'DisplayName','Healthy windows');
scatter(all_ib_f, all_id_f, 30, [0.85 0.1 0.1], 'x', 'DisplayName','Fault windows');

% Characteristic curve
plot(ib_curve, id_curve, 'k-', 'LineWidth', 2.0, 'DisplayName', ...
    sprintf('Dual-Slope (S1=%.2f, S2=%.2f, Pu=%.2f, Knee=%.1f)', ...
    SLOPE1_PLOT, SLOPE2_PLOT, PICKUP_PLOT, KNEE_PLOT));

% Knee point marker
knee_id = dualSlopeThr(KNEE_PLOT, SLOPE1_PLOT, PICKUP_PLOT, KNEE_PLOT, SLOPE2_PLOT, OFFSET2_PLOT);
plot(KNEE_PLOT, knee_id, 'ko', 'MarkerSize', 8, 'MarkerFaceColor','k', ...
    'HandleVisibility','off');
text(KNEE_PLOT + 0.3, knee_id, sprintf('Knee (%.0fA, %.2fA)', KNEE_PLOT, knee_id), ...
    'FontSize', 9);

% TRIP / SAFE labels
text(ib_range(2)*0.65, min(id_curve)*0.3, 'SAFE (No Trip)', ...
    'FontSize', 12, 'Color', [0.2 0.5 0.2], 'FontWeight', 'bold');
text(ib_range(2)*0.1, max(id_curve)*0.8, 'TRIP Zone', ...
    'FontSize', 12, 'Color', [0.7 0 0], 'FontWeight', 'bold');

xlabel('I_{bias} (A)', 'FontSize', 12);
ylabel('I_{diff} (A)', 'FontSize', 12);
title(sprintf('Differential Bias Characteristic — %d Healthy + %d Fault windows (%d files)', ...
    numel(all_ib_h), numel(all_ib_f), n_files), 'FontSize', 12);
legend('Location','northwest','FontSize', 9);

xlim(ib_range);
ylim([0, max([all_id_h, all_id_f]) * 1.10]);

out_scatter = fullfile(out_dir, 'bias_scatter.png');
exportgraphics(fig, out_scatter, 'Resolution', 150);
close(fig);
fprintf('Scatter plot saved: %s\n\n', out_scatter);

% =========================================================================
% 7.  GRID SEARCH
% =========================================================================
fprintf('--- Pass 2: Grid Search ---\n');

% Build combined dataset for fast scoring
ib_all = [all_ib_h, all_ib_f];
id_all = [all_id_h, all_id_f];
zone_h = [true(1, numel(all_ib_h)), false(1, numel(all_ib_f))];
zone_f = ~zone_h;

n_h = sum(zone_h);  n_f = sum(zone_f);

% Result storage: [pickup, s1, s2, knee, fp_pct, det_pct, min_h_margin, min_f_margin]
results = [];

total_combos = numel(gs_pickup)*numel(gs_slope1)*numel(gs_slope2)*numel(gs_knee);
combo_count  = 0;

for pu = gs_pickup
    for s1 = gs_slope1
        for s2 = gs_slope2
            if s2 <= s1, continue; end   % slope2 must exceed slope1 (physically)
            for kn = gs_knee
                combo_count = combo_count + 1;
                off2 = (s2 - s1) * kn - pu;  % continuity condition

                % Vectorised threshold computation
                thr = zeros(1, numel(ib_all));
                lo  = ib_all < kn;
                thr( lo) = s1 * ib_all( lo) + pu;
                thr(~lo) = s2 * ib_all(~lo) - off2;

                trip = id_all > thr;

                fp_count  = sum(trip & zone_h);
                det_count = sum(trip & zone_f);

                fp_pct  = 100 * fp_count  / n_h;
                det_pct = 100 * det_count / n_f;

                % Margins (positive = good)
                margin_h = min(thr(zone_h) - id_all(zone_h));   % healthy: want thr > id → positive
                margin_f = min(id_all(zone_f) - thr(zone_f));   % fault:   want id > thr → positive

                results(end+1, :) = [pu, s1, s2, kn, fp_pct, det_pct, margin_h, margin_f]; %#ok<AGROW>
            end
        end
    end
end

fprintf('Evaluated %d/%d valid combinations.\n', size(results,1), total_combos);

% --- Filter: FP=0 AND Detection=100 ---
pass_mask = (results(:,5) == 0) & (results(:,6) == 100);
n_pass    = sum(pass_mask);
fprintf('Combinations with FP=0%% AND Det=100%%: %d\n\n', n_pass);

% --- Sort passing results by composite margin (min of H and F margin) ---
pass_results = results(pass_mask, :);
if ~isempty(pass_results)
    composite_margin = min(pass_results(:,7), pass_results(:,8));
    [~, sort_idx]    = sort(composite_margin, 'descend');
    pass_results     = pass_results(sort_idx, :);
end

% --- Sort ALL results by detection rate desc, then fp asc, then margin ---
all_sort_key = results(:,6) - results(:,5)/100;
[~, all_sort] = sort(all_sort_key, 'descend');
results_sorted = results(all_sort, :);

% =========================================================================
% 8.  WRITE GRID SEARCH CSV
% =========================================================================
fid_gs = fopen(fullfile(out_dir, 'grid_search_results.csv'), 'w');
fprintf(fid_gs, 'PICKUP,SLOPE1,SLOPE2,KNEE_POINT,OFFSET2,FP_pct,Det_pct,MinMargin_H,MinMargin_F,CompositeMargin,PASS\n');

for k = 1:size(results_sorted,1)
    r    = results_sorted(k,:);
    off2 = (r(3)-r(2))*r(4) - r(1);
    comp = min(r(7), r(8));
    pass_str = ternary((r(5)==0) && (r(6)==100), 'PASS', 'FAIL');
    fprintf(fid_gs, '%.3f,%.3f,%.3f,%.1f,%.4f,%.2f,%.2f,%.4f,%.4f,%.4f,%s\n', ...
        r(1),r(2),r(3),r(4), off2, r(5),r(6), r(7),r(8), comp, pass_str);
end
fclose(fid_gs);
fprintf('Grid search CSV saved: %s\n', fullfile(out_dir,'grid_search_results.csv'));

% =========================================================================
% 9.  WRITE BEST PARAMETERS (firmware-ready)
% =========================================================================
fid_best = fopen(fullfile(out_dir, 'grid_search_best.txt'), 'w');

if ~isempty(pass_results)
    best = pass_results(1,:);
    off2_best = (best(3)-best(2))*best(4) - best(1);

    fprintf(fid_best, '=== Grid Search Best Result ===\n');
    fprintf(fid_best, 'FP=0%%%% AND Detection=100%%%% passing sets: %d\n\n', n_pass);
    fprintf(fid_best, 'Best (max composite margin = %.4f A):\n', min(best(7),best(8)));
    fprintf(fid_best, '  PICKUP     = %.3f A\n', best(1));
    fprintf(fid_best, '  SLOPE1     = %.3f\n',   best(2));
    fprintf(fid_best, '  SLOPE2     = %.3f\n',   best(3));
    fprintf(fid_best, '  KNEE_POINT = %.1f A\n', best(4));
    fprintf(fid_best, '  OFFSET2    = %.4f A  (calculated: (S2-S1)*Knee - Pickup)\n', off2_best);
    fprintf(fid_best, '  Margin_H   = %.4f A  (healthy safety margin)\n', best(7));
    fprintf(fid_best, '  Margin_F   = %.4f A  (fault detection margin)\n', best(8));
    fprintf(fid_best, '\n--- Copy to firmware (.ino) ---\n');
    fprintf(fid_best, 'const float SLOPE1     = %.3ff;\n', best(2));
    fprintf(fid_best, 'const float PICKUP     = %.3ff;\n', best(1));
    fprintf(fid_best, 'const float KNEE_POINT = %.1ff;\n', best(4));
    fprintf(fid_best, 'const float SLOPE2     = %.3ff;\n', best(3));
    fprintf(fid_best, 'const float OFFSET2    = %.4ff;\n', off2_best);

    % Also print to console
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════╗\n');
    fprintf('║         BEST PARAMETERS (FP=0, Det=100)      ║\n');
    fprintf('╠══════════════════════════════════════════════╣\n');
    fprintf('║  PICKUP     = %.3f A                         ║\n', best(1));
    fprintf('║  SLOPE1     = %.3f                           ║\n', best(2));
    fprintf('║  SLOPE2     = %.3f                           ║\n', best(3));
    fprintf('║  KNEE_POINT = %.1f A                         ║\n', best(4));
    fprintf('║  OFFSET2    = %.4f A                         ║\n', off2_best);
    fprintf('║  Margin_H   = %.4f A                         ║\n', best(7));
    fprintf('║  Margin_F   = %.4f A                         ║\n', best(8));
    fprintf('║  Passing sets: %-4d / %-4d evaluated         ║\n', n_pass, total_combos);
    fprintf('╚══════════════════════════════════════════════╝\n\n');
else
    fprintf(fid_best, 'NO combination achieved FP=0%% AND Detection=100%%.\n');
    fprintf(fid_best, 'Top 5 by detection rate:\n\n');
    for k = 1:min(5, size(results_sorted,1))
        r    = results_sorted(k,:);
        off2 = (r(3)-r(2))*r(4) - r(1);
        fprintf(fid_best, '  FP=%.1f%% Det=%.1f%%  PU=%.3f S1=%.3f S2=%.3f Knee=%.1f OFFSET2=%.4f\n', ...
            r(5),r(6), r(1),r(2),r(3),r(4), off2);
    end
    fprintf('\n[WARNING] No perfect combination found — consider widening grid search range.\n\n');
end
fclose(fid_best);
fprintf('Best parameters saved: %s\n', fullfile(out_dir,'grid_search_best.txt'));

fprintf('\n=== Done. All outputs in %s/ ===\n', out_dir);
fprintf('  bias_characteristic.csv   — raw I_diff/I_bias per window\n');
fprintf('  bias_scatter.png          — scatter plot (tune SECTION 1 and re-run to adjust curve)\n');
fprintf('  grid_search_results.csv   — all evaluated combinations\n');
fprintf('  grid_search_best.txt      — firmware-ready best parameters\n');

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function row = findTransition(I1, win)
N = length(I1);
best_row = win+1;  best_ratio = 0;
for i = win+1 : N-win
    rb = sqrt(mean(I1(i-win : i-1).^2));
    ra = sqrt(mean(I1(i     : i+win-1).^2));
    r  = (rb > 1e-9) * ra / max(rb, 1e-9);
    if r > best_ratio, best_ratio = r; best_row = i; end
end
row = best_row;
end

function thr = dualSlopeThr(ib, s1, pu, knee, s2, off2)
if ib < knee
    thr = s1 * ib + pu;
else
    thr = s2 * ib - off2;
end
end

function s = ternary(cond, a, b)
if cond, s = a; else, s = b; end
end


