%% Plot for presentation
clear all, close all; clc;
load('project_data.mat') %load the path file
load('desired.mat');
path.UxDes = Ux_des;
path.axDes = ax_des;
save('project_data.mat','edge_list','path');