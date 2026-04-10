% plot_signals.m
% Plot I1 (primary) and I4 (secondary) with Healthy/Fault regions
% Output: Signal_Plots/<GroupName>_<filename>.png  (flat, no subfolders)

base_dir = 'Selected_Signals';
out_dir  = 'Signal_Plots';
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

groups = dir(base_dir);
groups = groups([groups.isdir] & ~startsWith({groups.name}, '.'));

total_saved = 0;

for g = 1:length(groups)
    group_path = fullfile(base_dir, groups(g).name);
    files = dir(fullfile(group_path, '*.csv'));

    for f = 1:length(files)
        fpath = fullfile(group_path, files(f).name);
        fname = files(f).name;
        gname = groups(g).name;

        % Read CSV
        try
            data = readmatrix(fpath, 'NumHeaderLines', 1);
        catch
            fprintf('SKIP (read error): %s\n', fpath);
            continue
        end

        if size(data,1) < 400 || size(data,2) < 11
            fprintf('SKIP (too small): %s\n', fpath);
            continue
        end

        I1 = data(:, 8);    % Primary current
        I4 = data(:, 11);   % Secondary current
        N  = length(I1);
        t  = (1:N)';

        % --- Find transition row via sliding RMS window (on I1) ---
        win = 200;
        best_row   = win + 1;
        best_ratio = 0;

        for i = (win+1):(N-win)
            rms1 = sqrt(mean(I1(i-win:i-1).^2));
            rms2 = sqrt(mean(I1(i:i+win-1).^2));
            if rms1 > 0
                r = rms2 / rms1;
                if r > best_ratio
                    best_ratio = r;
                    best_row   = i;
                end
            end
        end

        buf         = 300;
        healthy_end = max(1, best_row - buf);
        fault_start = min(N, best_row + buf);

        % --- Axis limits (cover both I1 and I4) ---
        all_vals = [I1; I4];
        y_min = min(all_vals);
        y_max = max(all_vals);
        y_pad = (y_max - y_min) * 0.15;
        y_lo  = y_min - y_pad;
        y_hi  = y_max + y_pad;

        % --- Plot ---
        fig = figure('Visible', 'off', 'Position', [0 0 1400 550]);
        hold on;

        % Shaded regions
        fill([1, healthy_end, healthy_end, 1], ...
             [y_lo, y_lo, y_hi, y_hi], ...
             [0.6 1.0 0.6], 'FaceAlpha', 0.20, 'EdgeColor', 'none');

        fill([fault_start, N, N, fault_start], ...
             [y_lo, y_lo, y_hi, y_hi], ...
             [1.0 0.6 0.6], 'FaceAlpha', 0.20, 'EdgeColor', 'none');

        fill([healthy_end, fault_start, fault_start, healthy_end], ...
             [y_lo, y_lo, y_hi, y_hi], ...
             [1.0 1.0 0.6], 'FaceAlpha', 0.30, 'EdgeColor', 'none');

        % Signals
        plot(t, I1, 'b-', 'LineWidth', 0.7);
        plot(t, I4, 'Color', [0.85 0.33 0.10], 'LineWidth', 0.7);

        % Transition line
        xline(best_row, 'r--', 'LineWidth', 1.5);

        % Region labels
        y_label = y_hi - y_pad * 0.5;
        if healthy_end > 200
            text(healthy_end/2, y_label, 'HEALTHY', ...
                 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.45 0], ...
                 'HorizontalAlignment', 'center');
        end
        if (N - fault_start) > 200
            text((fault_start + N)/2, y_label, 'FAULT', ...
                 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.7 0 0], ...
                 'HorizontalAlignment', 'center');
        end
        text(best_row, y_hi, sprintf(' row=%d', best_row), ...
             'FontSize', 8, 'Color', 'r', 'HorizontalAlignment', 'left');

        hold off;

        xlabel('Sample Index');
        ylabel('Current (A)');
        title(sprintf('%s  /  %s    |    jump ratio = %.2f', ...
              strrep(gname,'_',' '), strrep(fname,'.csv',''), best_ratio), ...
              'Interpreter', 'none', 'FontSize', 11);
        legend({'Healthy zone','Fault zone','Transition buffer', ...
                'I1 (Primary)','I4 (Secondary)','Transition'}, ...
               'Location', 'northwest', 'FontSize', 8);
        grid on;
        ylim([y_lo, y_hi]);
        xlim([1, N]);

        % Save — flat folder, filename = GroupName_originalname.png
        safe_gname = strrep(gname, ' ', '_');
        out_name   = sprintf('%s_%s', safe_gname, strrep(fname, '.csv', '.png'));
        out_path   = fullfile(out_dir, out_name);
        exportgraphics(fig, out_path, 'Resolution', 150);
        close(fig);

        total_saved = total_saved + 1;
        fprintf('SAVED [%3d]: %s\n', total_saved, out_path);
    end
end

fprintf('\n=== Done. %d plots saved to %s/ ===\n', total_saved, out_dir);
