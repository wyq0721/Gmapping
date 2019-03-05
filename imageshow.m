%% Script description: Test for image show
%===============================================================================
% INPUT:
% @data         the data resulted from Gmapping
% OUTPUT:
% map           show the complete map
% DATE:         2018/12/24 wyq
%===============================================================================


%% workshop
% clear 
% clc
% load map_workshop.mat;
% real_pose =[final_pose(1,:)*params.grid_size-params.world_center(1);final_pose(2,:)*params.grid_size-params.world_center(2)];
% %     plot(real_pose(1,:),real_pose(2,:),'.r')
% image_pose = [(real_pose(1,:)+params.world_center(1))./params.grid_size;-(real_pose(2,:)+params.world_center(2)- params.world_size(1))./params.grid_size];
% mp = 1- particles.map.occupy;
% mp =  flipud(mp);
% figure(1)
% clf;
% imshow(mp(),[])
% hold on
% plot(image_pose(1,:),image_pose(2,:),'.r')
% axis on 
% xstart = (params.world_center(1)-10)/params.grid_size;
% xend = (params.world_center(1)+50)/params.grid_size;
% xtick_interval = 10;
% ystart = (params.world_size(1)-params.world_center(2)-15)/params.grid_size;
% yend = (params.world_size(1)-params.world_center(2)+15)/params.grid_size;
% ytick_interval = 5;
% xlim([xstart xend])
% xticks(xstart:xtick_interval/params.grid_size:xend)
% xticklabels({'-10','0','10','20','30','40','50'})
% ylim([ystart yend])
% yticks(ystart:ytick_interval/params.grid_size:xend)
% yticklabels({'15','10','5','0','-5','-10','-15'})


%% seattle
clear 
clc
% load map_seattle;
load map_seattle15.mat;
final_pose = world2map(particles(max_i).p(1:2,:),params);
real_pose =[final_pose(1,:)*params.grid_size-params.world_center(1);final_pose(2,:)*params.grid_size-params.world_center(2)];
%     plot(real_pose(1,:),real_pose(2,:),'.r')
image_pose = [(real_pose(1,:)+params.world_center(1))./params.grid_size;-(real_pose(2,:)+params.world_center(2)- params.world_size(1))./params.grid_size];
mp = 1- particles(max_i).map.occupy;
mp =  flipud(mp);
figure(2)
clf
imshow(mp(),[])
hold on
plot(image_pose(1,:),image_pose(2,:),'.r')
axis on 
xstart = (params.world_center(1)-15)/params.grid_size;
xend = (params.world_center(1)+20)/params.grid_size;
xtick_interval = 5;
ystart = (params.world_size(1)-params.world_center(2)-10)/params.grid_size;
yend = (params.world_size(1)-params.world_center(2)+25)/params.grid_size;
ytick_interval = 5;
xlim([xstart xend])
xticks(xstart:xtick_interval/params.grid_size:xend)
xticklabels({'-15','-10','-5','0','5','10','15','20'})% from left to right
ylim([ystart yend])
yticks(ystart:ytick_interval/params.grid_size:xend)
yticklabels({'10','5','0','-5','-10','-15','-20','-25'})% from up to down
set(gca,'tickdir','in');
title('Gmapping of Seattle Intel Research Lab')


