% 設定檔案路徑
file_path = 'C:\Users\ASUS\Desktop\碩士論文\Data\Goal.xlsx';

% 定義工作表名稱與繪圖參數
sheet_names = {'obstacle to goal', 'obstacle to start', 'long path'};
colors = {'b', 'b', 'b'};        
markers = {'o', 'o', 'o'};     

% 依序處理每個工作表
for i = 1:length(sheet_names)
    % 讀取資料，只取第 2、3 欄
    data = readmatrix(file_path, 'Sheet', sheet_names{i});
    
    % 資料安全檢查
    if size(data, 2) < 3
        warning('工作表 "%s" 的欄位不足，略過此表。', sheet_names{i});
        continue;
    end

    y = data(:, 2);  % 水平座標 y（第二欄）
    x = data(:, 3);  % 垂直座標 x（第三欄）

    % 繪製單獨圖形
    figure;
    scatter(y, x, 80, colors{i}, markers{i}, 'filled');
    grid on;
    axis equal;
    title(sprintf('Position Distribution: %s', sheet_names{i}));
    xlabel('Y (cm)');
    ylabel('X (cm)');
end
