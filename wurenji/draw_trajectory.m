output_x = out.yout{1}.Values.;
output_y =out.yout{2}.Values;
output_z = out.yout{3}.Values;

figure('Color','w');  % 创建白色背景图窗
scatter3(output_x, output_y, output_z, ...  % 三维散点图核心函数
         50, ...       % 点的大小（可调整，越大点越明显）
         'filled', ... % 点填充（实心点，更清晰）
         'r');         % 点的颜色（红色，可改为'b'蓝色、'g'绿色等）

% 5. 图形美化与标签
xlabel('X 轴 (m)','FontSize',12);  % x轴标签
ylabel('Y 轴 (m)','FontSize',12);  % y轴标签
zlabel('Z 轴 (m)','FontSize',12);  % z轴标签
title('三维圆形轨迹散点图','FontSize',14,'FontWeight','bold');  % 标题
grid on;  % 显示网格（便于观察三维结构）
axis equal;  % 三轴等比例显示（避免图形拉伸）
view(45, 30);  % 设置视角（ azimuth=45°, elevation=30°，可调整）