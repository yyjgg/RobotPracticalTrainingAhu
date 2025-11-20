output_x = out.yout{1}.Values.Data;
output_y =out.yout{2}.Values.Data;
output_z = out.yout{3}.Values.Data;
numPoints = height(output_z);
colors = hsv(numPoints); 
figure('Color','w');  % 创建白色背景图窗
% 5. 图形美化与标签
xlabel('X 轴 (m)','FontSize',12);  % x轴标签
ylabel('Y 轴 (m)','FontSize',12);  % y轴标签
zlabel('Z 轴 (m)','FontSize',12);  % z轴标签
title('三维圆形轨迹散点图','FontSize',14,'FontWeight','bold');  % 标题
grid on;  % 显示网格（便于观察三维结构）
axis equal;  % 三轴等比例显示（避免图形拉伸）
view(45, 30);  % 设置视角（ azimuth=45°, elevation=30°，可调整）
for i = 1:numPoints

scatter3(output_x(i), output_y(i), output_z(i), ...  % 三维散点图核心函数
         50, ...       % 点的大小（可调整，越大点越明显
         colors(i,:));         % 点的颜色（红色，可改为'b'蓝色、'g'绿色等）


hold on;
 pause(0.01);
end


