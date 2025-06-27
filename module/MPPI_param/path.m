folder_path = 'data/Path/';  % 檔案資料夾
num_files = 9;

figure;
hold on;

for i = 1:num_files
    file_path = fullfile(folder_path, sprintf('%d.csv', i));
    fprintf('Processing file: %s\n', file_path);
    
    try
        data = readtable(file_path, 'ReadVariableNames', false);
    catch ME
        warning('讀取檔案 %s 失敗: %s', file_path, ME.message);
        continue;
    end

    % 確認至少有兩欄 (x,y)
    if size(data, 2) < 2
        warning('檔案 %d.csv 欄位不足，跳過', i);
        continue;
    end

    % 繪製軌跡 (x,y)
    plot(data{:,1}, data{:,2}, 'LineWidth', 1.5, 'DisplayName', sprintf('File %d', i));
end

xlabel('X (m)');
ylabel('Y (m)');
title('Robot Trajectories');
legend show;
grid on;
axis equal;
