%% Script description: Gmapping example
%===============================================================================
% INPUT:
% @exmaple      1 or 2 corresponding to different dataset
% OUTPUT:
% map           show the SLAM progress
% DATE:         2018/12/23 wyq
% DATE2:        2018/12/24 wyq add imagine show and examples choice in version 2
%===============================================================================

close all;
clear;
clc;

% Change this number to try the different examples provided
example = 1;  
switch example
  case 1      
    DatasetName = 'seattle';  % Indoor
  case 2      
    DatasetName = 'workshop';  % Indoor
end

if strcmp(DatasetName, 'seattle')
    %% dataset 1 'seattle'
        
    % load dataset
    load seattle.mat
    data.laser = range;
    data.odometry = odom;
    pose = zeros(2,2000);
        
    % particles filter parameters
    params.particles_size = 1;
    
    % laser sensor parameters
    params.max_angle = 90/180*pi;
    params.num_beams = 180;
    params.max_range = 14;
    params.usable_range = [0.2,14];
    
    % threshold parameters for efficiency
    params.m_linearDistance = 0;
    params.m_angularDistance = 0;
    params.m_linearThresholdDistance = 0;
    params.m_angularThresholdDistance = 0;
    
    % motion model parameters (if only 1 particle, set 0)
    params.srr = 0;
    params.srt = 0;
    params.str = 0;
    params.stt = 0;
    
    % scan matching parameters
    params.move_step = 0.8;
    params.rotate_step = 0.4;
    params.refinement_times = 12;
    % in function 'scan_score'
    params.gaussian_sigma = 0.05;
    params.occupied_threshold = 0.8;
    params.hit_kernel = [0,-1,0,0,1,-1,-1,1,1,-2,0,0,2,-2,-2,-1,-1,1,1,2,2,-2,-2,2,2,-3,0,0,3,-3,-3,-1,-1,1,1,3,3,-3,-3,-2,-2,2,2,3,3,-4,0,0,4;...
        0,0,-1,1,0,-1,1,-1,1,0,-2,2,0,-1,1,-2,2,-2,2,-1,1,-2,2,-2,2,0,-3,3,0,-1,1,-3,3,-3,3,-1,1,-2,2,-3,3,-3,3,-2,2,0,-4,4,0];
    
    % weight update parameters
    params.obs_sigma_gain = 1;
    params.resample_threshold = 0.5;
    
    % map update parameters
    params.grid_size = 0.1;
    params.world_size = [70,70];
    params.world_center = [35,45];
    params.hit_weight = log(0.8/0.2);

    % prepare the space
    particles.p = zeros(3,size(pose,2));
    particles.w = 1/params.particles_size;
    particles.map.hit = zeros(params.world_size/params.grid_size);
    particles.map.visit = zeros(params.world_size/params.grid_size);
    particles.map.occupy = 0.5*ones(params.world_size/params.grid_size);
    particles = repmat(particles,[params.particles_size 1]);
    
    step=0;
    % usage example
    while(step< size(pose,2))

       % main process
        tic;
        [step, particles] = process_scan_main_loop(step, particles, data,params);
        t2 = toc;
        max_w =0;
        for i = 1: params.particles_size
            if  particles(i).w > max_w
                max_w = particles(i).w;
                max_i = i;
            end
        end

        % plot
        imshow(flipud(1 - particles(max_i).map.occupy),[]);
        set(gcf, 'position', [400 100 800 600])   
        colormap(gray);  
        hold on 
        pose(:, step) = world2map(particles(max_i).p(1:2,step), params);
        plot(pose(1, 1:step), params.world_size(1) / params.grid_size - pose(2, 1:step), '-b', 'linewidth', 3);
        hold off
        grid on 

        % gca set   
        axis on
        xstart = (params.world_center(1) - 15) / params.grid_size;
        xend = (params.world_center(1) + 20) / params.grid_size;
        xtick_interval = 5;
        ystart = (params.world_size(1) - params.world_center(2) - 10) / params.grid_size;
        yend = (params.world_size(1) - params.world_center(2) + 25) / params.grid_size;
        ytick_interval = 5;
        xlim([xstart xend])
        xticks(xstart: xtick_interval / params.grid_size: xend)
        xticklabels({'-15','-10','-5','0','5','10','15','20'})% from left to right
        ylim([ystart yend])
        yticks(ystart: ytick_interval / params.grid_size: xend)
        yticklabels({'10','5','0','-5','-10','-15','-20','-25'})% from up to down
        set(gca, 'FontSize', 12, 'FontWeight', 'Bold', 'TickDIR', 'in')

        %text display
        text(210, 175, 'Gmapping', 'Color', 'blue', 'FontSize', 13)
        text(475, 175, 'Step', 'Color', 'blue', 'FontSize', 13)
        text(475, 185, 'Time', 'Color', 'blue', 'FontSize', 13)
        text(505, 175, num2str(step), 'Color', 'blue', 'FontSize',13)
        text(505, 185, num2str(t2), 'Color', 'blue', 'FontSize', 13)
        drawnow 
    end


else 
    %% dataset 2 'workshop'
    
    % load dataset
    load workshop.mat urglog;
    data.laser = urglog;
    data.odometry = ones(size(urglog,1), 3);
    pose = zeros(2, size(urglog,1));  
    
    % particles filter parameters
    params.particles_size = 1;
    
    % laser sensor parameters
    params.max_angle = 135/180*pi;
    params.num_beams = 1081;
    params.max_range = 20;
    params.usable_range = [0.2,20];
    
    % threshold parameters for efficiency
    params.m_linearDistance = 0;
    params.m_angularDistance = 0;
    params.m_linearThresholdDistance = 0;
    params.m_angularThresholdDistance = 0;
    
    % motion model parameters (if only 1 particle, set 0)
    params.srr = 0;
    params.srt = 0;
    params.str = 0;
    params.stt = 0;
    
    % scan matching parameters
    params.move_step = 0.8;
    params.rotate_step = 0.4;
    params.refinement_times = 12;
    % in function 'scan_score'
    params.gaussian_sigma = 0.05;
    params.occupied_threshold = 0.8;
    params.hit_kernel = [0,-1,0,0,1,-1,-1,1,1,-2,0,0,2,-2,-2,-1,-1,1,1,2,2,-2,-2,2,2,-3,0,0,3,-3,-3,-1,-1,1,1,3,3,-3,-3,-2,-2,2,2,3,3,-4,0,0,4;...
        0,0,-1,1,0,-1,1,-1,1,0,-2,2,0,-1,1,-2,2,-2,2,-1,1,-2,2,-2,2,0,-3,3,0,-1,1,-3,3,-3,3,-1,1,-2,2,-3,3,-3,3,-2,2,0,-4,4,0];
    
    % weight update parameters
    params.obs_sigma_gain = 1;
    params.resample_threshold = 0.5;
    
    % map update parameters
    params.grid_size = 0.1;
    params.world_size = [40,70];
    params.world_center = [10,20];
    params.hit_weight = log(0.8/0.2);
    
    % prepare the space    
    particles.p = zeros(3, size(pose,2));
    particles.w = 1 / params.particles_size;
    particles.map.hit = zeros(params.world_size / params.grid_size);
    particles.map.visit = zeros(params.world_size / params.grid_size);
    particles.map.occupy = 0.5*ones(params.world_size / params.grid_size);
    particles = repmat(particles, [params.particles_size 1]);

    step=0;    
    % usage example
    while(step< size(pose,2))
        
        % main process
        tic;
        [step,particles] = process_scan_main_loop(step, particles,data,params);
        t2 = toc;
        max_w = 0;
        for i = 1: params.particles_size
            if  particles(i).w > max_w
                max_w = particles(i).w;
                max_i = i;
            end
        end
        
        % plot
        imshow(flipud(1-particles(max_i).map.occupy), []);
        set(gcf, 'position', [100 100 1200 600])   
        colormap(gray);  
        hold on 
        pose(:, step) = world2map(particles(max_i).p(1:2,step), params);
        plot(pose(1,1:step), params.world_size(1) / params.grid_size - pose(2,1:step), '-b', 'linewidth', 3);
        hold off
        grid on 
        
        % gca set
        axis on
        xstart = (params.world_center(1)-10) / params.grid_size;
        xend = (params.world_center(1)+50) / params.grid_size;
        xtick_interval = 10;
        ystart = (params.world_size(1)-params.world_center(2)-15) / params.grid_size;
        yend = (params.world_size(1)-params.world_center(2)+15) / params.grid_size;
        ytick_interval = 5;
        xlim([xstart xend])
        xticks(xstart: xtick_interval / params.grid_size: xend)
        xticklabels({'-10','0','10','20','30','40','50'})
        ylim([ystart yend])
        yticks(ystart: ytick_interval / params.grid_size: xend)
        yticklabels({'15','10','5','0','-5','-10','-15'})
        set(gca, 'FontSize', 12, 'FontWeight', 'Bold', 'TickDIR', 'in')
        
        %text display
        text(20, 75, 'Gmapping', 'Color', 'blue', 'FontSize', 13)
        text(20, 320, 'Step', 'Color', 'blue',  'FontSize', 13)
        text(20, 330, 'Time', 'Color','blue', 'FontSize', 13)
        text(50, 320, num2str(step), 'Color', 'blue','FontSize', 13)
        text(50, 330, num2str(t2), 'Color', 'blue','FontSize', 13)
        drawnow 
    end


end 
