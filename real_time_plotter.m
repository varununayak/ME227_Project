%% Real Time Plotter For 227 Project
clear all; close all; clc

%load('project_lookahead05.mat');
load('project_pid05.mat');
%load('project_lqr05.mat');

load('project_data.mat')
realpath.s_m = s_m;
realpath.k_1pm = k_1pm;
realpath.psi_rad = path.psi_rad + dpsi_rad;
realpath.posE_m = East_m;
realpath.posN_m = North_m;

path.s = path.s_m;
path.k = path.k_1pm;
path.posE = path.posE_m;
path.posN = path.posN_m;
path.psi = path.psi_rad;
setup_niki;
speed = sqrt(Ux_mps.^2 + Uy_mps.^2);

animate2(path, veh, dpsi_rad, s_m, e_m, delta_rad, speed)



