figure;
title("Rotation Error between Two LiDARs",'fontname','times new roman')
hold on;
load("simu_data/noise-free-result.mat");
plot(mean(aver_theta,1),'.-r');
plot(median(aver_theta,1),'.--r');
hold on;
load("simu_data/noise-0.015-result.mat");
plot(mean(aver_theta,1),'.-g');
plot(median(aver_theta,1),'.--g');
load("simu_data/noise-0.03-result.mat");
plot(mean(aver_theta,1),'.-b');
plot(median(aver_theta,1),'.--b');
ylabel("Rotation Rrror(Degree)",'fontname','times new roman');
xlabel("The number of poses",'fontname','times new roman');
legend("mean:\sigma_L=0","median:\sigma_L=0",...
       "mean:\sigma_L=0.015","median:\sigma_L=0.015",...
       "mean:\sigma_L=0.03","median:\sigma_L=0.03",'fontname','times new roman');

figure;
title("Translation Error between Two LiDARs",'fontname','times new roman')
hold on;
load("simu_data/noise-free-result.mat");
plot(mean(aver_t_err,1),'.-r');
plot(median(aver_t_err,1),'.--r');
hold on;
load("simu_data/noise-0.015-result.mat");
plot(mean(aver_t_err,1),'.-g');
plot(median(aver_t_err,1),'.--g');
load("simu_data/noise-0.03-result.mat");
plot(mean(aver_t_err,1),'.-b');
plot(median(aver_t_err,1),'.--b');

ylabel("Translation Rrror(Meter)",'fontname','times new roman');
xlabel("The number of poses",'fontname','times new roman');
legend("mean:\sigma_L=0","median:\sigma_L=0",...
       "mean:\sigma_L=0.015","median:\sigma_L=0.015",...
       "mean:\sigma_L=0.03","median:\sigma_L=0.03",'fontname','times new roman');
