clc, clear;
close all;

%% 
syms rz1 ry1 rx1 rz2 ry2 rx2
% syms tx1 ty1 tz1 tx2 ty2 tz2
syms tx1 ty1 tz1 
tx2 = -tx1;
ty2 = -ty1;
tz2 = -tz1;
syms fx fy cx cy
R1 = [cos(rz1), -sin(rz1) 0; sin(rz1) cos(rz1), 0; 0 0 1] ... 
  * [cos(ry1) 0 sin(ry1); 0 1 0; -sin(ry1) 0 cos(ry1)] ...
  * [1 0 0; 0 cos(rx1) -sin(rx1); 0 sin(rx1) cos(rx1)];
t1 = [tx1; ty1; tz1];

R2 = [cos(rz2), -sin(rz2) 0; sin(rz2) cos(rz2), 0; 0 0 1] ... 
  * [cos(ry2) 0 sin(ry2); 0 1 0; -sin(ry2) 0 cos(ry2)] ...
  * [1 0 0; 0 cos(rx2) -sin(rx2); 0 sin(rx2) cos(rx2)];
t2 = [tx2; ty2; tz2];

K = [fx, 0, cx; 0, fy, cy; 0 0 1];

%%
syms px py pz
Pw = [px; py; pz];
Pc1 = K * (R1 * Pw + t1);
Pc2 = K * (R2 * Pw + t2);

% equality
pc1 = Pc1(1:2) ./ Pc1(3);
pc2 = Pc2(1:2) ./ Pc2(3);