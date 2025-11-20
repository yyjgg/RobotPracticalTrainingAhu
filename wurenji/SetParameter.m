%parameter file
%下正，上负
clc; clear;
PowerUnit_CR = 1148;%
PowerUnit_wb = -141.4;%
PowerUnit_Tm = 0.02;%
ConEfficiency_drone_R = 0.225;%
ConEfficiency_drone_cT = 1.105e-05;%
ConEfficiency_drone_cM = 1.779e-07;%
PosDyna_Mass = 1.4;%
AttDyna_Ixx = 0.0211;%
AttDyna_Iyy = 0.0219;%
AttDyna_Izz = 0.0366;%
PosDyna_GravityAcc = 9.8;%
POSITION_Z_ZERO = 0;%
PWM_1 = 0;%0-1000
PWM_2 = 0;
PWM_3 = 0;
PWM_4 = 0;
PWM_ALL = 0;
%%% 圆形% 确定航轨点数目
num_points =100;
heigh = 3;% 设置高度
% 预先生成储存 x、y 位置的向量
x = zeros (num_points,1);
y = zeros (num_points,1);
z = ones (num_points,1);
% 存储期望的轨迹点
desired = zeros (num_points+1,4);
desired_pos_series = zeros (num_points+1,3);
desired_yaw = zeros (num_points+1,1);
desired_pos_series (1,:)=[0,0,heigh];
% 生成轨迹点
for j = 1:num_points
    x(j) = 10*cos(2*pi*j/num_points);
    y(j) = 10*sin(2*pi*j/num_points);
end

% 给轨迹点赋值
desired_pos_series (2:end,1) = x (:,1);
desired_pos_series (2:end,2) = y (:,1);
desired_pos_series (2:end,3) = z (:,1)*heigh;
desired (:,1:3) = desired_pos_series (:,:);
desired (:,4) = desired_yaw;

