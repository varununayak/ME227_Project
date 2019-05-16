%% Project Plotter
clear all, close all, clc

load('project_data.mat');

load('project_lookahead05.mat');
%load('project_pid05.mat');
%load('project_lqr05.mat');

load('simulation_results_lookahead.mat');
%load('simulation_results_pid.mat');
%load('simulation_results_lqr.mat');


figure();

subplot(3,1,1)
plot(s_m_sim,e_m_sim,s_m,e_m);
legend("Sim","Exp");
title("Lateral Error");

subplot(3,1,2)
plot(s_m_sim,dpsi_rad_sim,s_m,dpsi_rad);
legend("Sim","Exp");
title("Heading Error \Delta \psi");

subplot(3,1,3)
plot(s_m_sim,Ux_des,'r--',s_m_sim,Ux_mps_sim,s_m,Ux_mps);
legend("Desired","Sim","Exp");
title("Speed Tracking Performance");

  figure();
plot(s_m_sim,delta_rad_sim,s_m,delta_rad);
legend('Sim', 'Exp');
title("Steering Angle");
ylabel("\delta [rad]");
xlabel("s [m]");

figure()
plot(s_m_sim,Fxcmd_N_sim,s_m_sim,Fx_N_sim,s_m,Fxcmd_N);
legend("Desired","Sim","Exp");
title("Longitudinal Force Tracking Performance");
ylabel("F_x [N]");
xlabel("s [m]");


