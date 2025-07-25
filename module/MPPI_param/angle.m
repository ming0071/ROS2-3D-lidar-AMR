folder_path = 'data/singlePath/'; 
highlight_files = [1, 4, 5, 6]; % 要比較的檔案編號

figure;
hold on;

line_styles = {'-', '--', ':', '-.'}; % 為黑白印刷友善設計
colors = lines(length(highlight_files)); % 不同顏色

for idx = 1:length(highlight_files)
    i = highlight_files(idx);
    file_path = fullfile(folder_path, sprintf('%d.csv', i));
    
    try
        data = readtable(file_path, 'ReadVariableNames', false);
    catch ME
        warning('讀取檔案 %s 失敗: %s', file_path, ME.message);
        continue;
    end

    if size(data, 2) < 2
        warning('檔案 %d.csv 欄位不足，跳過', i);
        continue;
    end

    % 疊加繪製軌跡，使用不同線型與顏色
    plot(data{:,1}, data{:,2}, ...
         'LineStyle', line_styles{idx}, ...
         'Color', colors(idx,:), ...
         'LineWidth', 2, ...
         'DisplayName', sprintf('File %d', i));
end

xlabel('X (m)');
ylabel('Y (m)');
title('AMR Trajectories Comparison (Files 1, 4, 5, 6)');
legend show;
grid on;
axis equal;
