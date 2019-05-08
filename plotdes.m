%% Plot for presentation
load('project_data.mat') %load the path file
load('desired.mat');

ax_des(1) = ax_des(2);
ax_des(996:1009) = 0;

path.UxDes = Ux_des;
path.axDes = ax_des;
path.ayDes = path.k_1pm.*(path.UxDes.^2);

figure()
subplot(3,1,1)
plot(path.s_m,path.UxDes);
title("Desired Longitudinal Speed");
xlabel("s [m]");
ylabel("U_x [m/s]");

subplot(3,1,2);
plot(path.s_m,path.axDes,path.s_m,-4*ones(length(path.s_m),1),'r--',path.s_m,3*ones(length(path.s_m),1),'r--')
ylim([-5 5]);
xlabel("s [m]");
ylabel("a_x [m/s2]");
title("Desired Longitudinal Acceleration");



subplot(3,1,3);
plot(path.s_m,path.ayDes,path.s_m,4*ones(length(path.s_m),1),'r--');
ylim([-5 5]);
xlabel("s [m]");
ylabel("a_y [m/s2]");
title("Desired Lateral Acceleration");

