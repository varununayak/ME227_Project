%% Plot for presentation
clear all, close all; clc;
load('project_data.mat') %load the path file
load('desired.mat');
path.UxDes = smooth(Ux_des,30);
path.axDes = smooth(ax_des,20);
save('project_data.mat','edge_list','path');