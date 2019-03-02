clear;
clc;

tic;
t1 = clock;
%% seattle
load seattle.mat
data.laser=range;
data.odometry=odom;
final_pose=zeros(2,2000);

% laser parameters in scan score
params.max_angle=90/180*pi;
params.num_beams=180;
params.max_range=14;
params.usable_range=[0.2,14];
            
% map parameters in score function
params.grid_size=0.1;
params.world_size=[70,70];
params.world_center=[35,45];

% particles filter parameters
params.particles_size=1;
params.obs_sigma_gain=1;
params.resample_threshold=0.5;

%% workshop
% load workshop.mat urglog;
% data.laser = urglog;
% data.odometry = ones(size(urglog,1),3);
% final_pose=zeros(2,size(urglog,1));
% 
% % laser parameters in scan score
% params.max_angle=135/180*pi;
% params.num_beams=1081;
% params.max_range=20;
% params.usable_range=[1,20];
%             
% % map parameters in score function
% params.grid_size=0.1;
% params.world_size=[40,70];
% params.world_center=[10,20];
% 
% % particles filter parameters
% params.particles_size=1;
% params.obs_sigma_gain=1;
% params.resample_threshold=0.5;

%% initial 
% motion modle parameters 
params.srr=0;
params.srt=0;
params.str=0;
params.stt=0; 
% scan match parameters
params.move_step=0.8;
params.rotate_step=0.4;
params.refinement_times= 12;
          
% parameters in scan score function 
params.gaussian_sigma=0.05;
% params.lsigma=0.075;
%return of gen_kernel function, for fast scan score
params.hit_kernel=[0,-1,0,0,1,-1,-1,1,1,-2,0,0,2,-2,-2,-1,-1,1,1,2,2,-2,-2,2,2,-3,0,0,3,-3,-3,-1,-1,1,1,3,3,-3,-3,-2,-2,2,2,3,3,-4,0,0,4;0,0,-1,1,0,-1,1,-1,1,0,-2,2,0,-1,1,-2,2,-2,2,-1,1,-2,2,-2,2,0,-3,3,0,-1,1,-3,3,-3,3,-1,1,-2,2,-3,3,-3,3,-2,2,0,-4,4,0];

particles.p=zeros(3,size(final_pose,2));
particles.w=1/params.particles_size;
particles.map.hit=zeros(params.world_size/params.grid_size);
particles.map.visit=zeros(params.world_size/params.grid_size);
particles.map.occupy=0.5*ones(params.world_size/params.grid_size);
particles = repmat(particles,[params.particles_size 1]);
step=0;

params.m_linearDistance = 0;
params.m_angularDistance = 0;
params.m_linearThresholdDistance = 0;
params.m_angularThresholdDistance = 0;
%% usage example
while(step< size(final_pose,2))

    tic;
    [step,particles]=process_scan_main_loop(step,particles,data,params);
    t2 = toc;
    max_w =0;
    for i = 1: params.particles_size
        if  particles(i).w>max_w
            max_w=particles(i).w;
            max_i = i;
        end
    end
    
    imshow(flipud(1-particles(max_i).map.occupy),[]);
    colormap(gray);  
    hold on 
    final_pose(:,step) = world2map(particles(max_i).p(1:2,step),params);
    plot(final_pose(1,1:step),params.world_size(1)/params.grid_size-final_pose(2,1:step),'-b','linewidth',3);
    hold off
    grid on 
    axis equal

    t_all=etime(clock,t1)/60;
    text(5,20,'One Time','Color','red','FontSize',14)
    text(5,60,'All Time','Color','red','FontSize',14)
    text(5,100,'Step','Color','red','FontSize',14)
    text(120,20,num2str(t2),'Color','red','FontSize',14)
    text(120,60,num2str(t_all),'Color','red','FontSize',14)
    text(120,100,num2str(step),'Color','red','FontSize',14)
    drawnow 
end
