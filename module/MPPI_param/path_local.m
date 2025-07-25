% 讀取資料
data1 = readtable('data/singlePath/controller/DWB.csv');
data2 = readtable('data/singlePath/controller/MPPI.csv');

% 畫出軌跡
figure;
plot(data1.x, data1.y, 'r:', 'LineWidth', 2); hold on;
plot(data2.x, data2.y, 'b-', 'LineWidth', 2);

% 圖表標籤
xlabel('X (m)');
ylabel('Y (m)');
title('AMR Trajectories');
legend('DWB', 'MPPI');
grid on;
axis equal;