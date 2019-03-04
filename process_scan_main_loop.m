%% Function description: Main loop of Gmapping
%===============================================================================
% INPUT:
% @step             previous step 
% @particles        particles information 
% @data             dataset 
% @params           listing in the up one level
% OUTPUT:
% @step             current step 
% @particles        updated particles information 
% DATE:             2018/12/23 wyq
%===============================================================================

function [step,particles] = process_scan_main_loop(step, particles, data, params)

% step update
step = step+1;

% read new data
r_new = data.laser(step,:)';

% read new odometry
p_new = data.odometry(step,:)';

if(1==step)
    p_old = p_new;
else
    p_old = data.odometry(step-1,:)';
end

delta = absolute_difference(p_new, p_old);

move = p_new - p_old;
move(3) = atan2(sin(move(3)), cos(move(3)));
params.m_linearDistance = params.m_linearDistance + norm(move(1:2));
params.m_angularDistance = params.m_angularDistance + abs(move(3)); 
     
if (params.m_linearDistance >= params.m_linearThresholdDistance || params.m_angularDistance >= params.m_angularThresholdDistance )
    params.m_linearDistance = 0;
    params.m_angularDistance = 0;

    for i = 1:params.particles_size

        %% draw_from_motion; PR.122 Algorithm sample_motion_model_odometry(odometry,pose in particle)
        sxy = 0.3*params.srr;
        noisy_point = delta;
        noisy_point = noisy_point + [sample_gaussian(params.srr*abs(delta(1)) + params.str*abs(delta(3)) + sxy*abs(delta(2)));...
                                     sample_gaussian(params.srr*abs(delta(2)) + params.str*abs(delta(3)) + sxy*abs(delta(1)));...
                                     sample_gaussian(params.stt*abs(delta(3)) + params.srt*sqrt(delta(1)^2 + delta(2)^2))];
        noisy_point(3) = mod(noisy_point(3),2*pi);
        if (noisy_point(3) > pi)
            noisy_point(3) = noisy_point(3)- 2*pi;
        end

        % assignment
        if step ~= 1
            particles(i).p(:,step) = absolute_sum(particles(i).p(:,step-1), noisy_point);
        end 
        
        % the end of draw form motion


        %% scan_match
        if (1~=step)
            refinement = 0;
            move = [params.move_step,-params.move_step,0,0,0,0;...
                    0,0,params.move_step,-params.move_step,0,0;...
                    0,0,0,0,params.rotate_step,-params.rotate_step];

            best_pose = particles(i).p(:,step);
            current_score = scan_score(best_pose, r_new, particles(i).map, params);
            best_score = current_score;

            while (refinement < params.refinement_times)
                if best_score >= current_score
                    move = move*0.5;
                else
%                     refinement
                    break;
                end
                
                % the score max means the scan consistence with current map
                for j = 1 : size(move, 2)%  columns = 'maybe'states,  change particle pose
                    current_pose = best_pose + move(:,j);
                    current_score = scan_score(current_pose, r_new, particles(i).map, params);
                    if(current_score > best_score)
                        best_score = current_score;
                        best_pose = current_pose;
                    end
                end
                refinement = refinement + 1;
            end

            % assignment
            particles(i).p(:,step) = best_pose;
            particles(i).w = best_score;
        end
        % the end of scan match
    end


    %% update_weights
    if(1~=step)
      
        % find the max weight
        w_max = 0;
        w_sum = 0;
        for i = 1 : params.particles_size
            if particles(i).w > w_max
                w_max = particles(i).w;
            end
        end
        
        % recompute weight
        for i = 1 : params.particles_size
            particles(i).w = exp(1/ (params.obs_sigma_gain*params.particles_size) * (particles(i).w - w_max));%particles(i).w-w_max<0
            w_sum = w_sum + particles(i).w;
        end
        
        % normalize
        neff = 0;
        for i = 1 : params.particles_size
            particles(i).w = particles(i).w / w_sum;
            neff = neff + particles(i).w^2;
        end
        neff = 1 / neff;

        % resample
        if neff < params.resample_threshold * params.particles_size
%             step
%             neff
            index = zeros(1, params.particles_size);
            interval = 1 / params.particles_size;
            target = interval*rand(1);
            j = 0;
            w_sum = 0;
            for i = 1 : params.particles_size
                w_sum = w_sum + particles(i).w;
                while(w_sum > target)
                    j = j + 1;
                    index(j) = i;
                    target = target + interval;
                end
            end
            
            new_particles = repmat(particles,[params.particles_size 1]);
            for i = 1 : length(index)
                new_particles(i) = particles(index(i));
            end
            for i = 1 : length(index)
                particles(i) = new_particles(i);
            end
            
            %% update
            for i = 1 : params.particles_size
                particles(i).w = 1 / params.particles_size;
            end

        end
        % the end of resample
    end
    % the end of update weight

    %% map update(register_scan)
    for i = 1 : params.particles_size
        a = linspace(-params.max_angle,params.max_angle, params.num_beams);
        p0 = particles(i).p(1:2,step);
        for j = 1:params.num_beams
            
            d = r_new(j);
            if d > params.max_range
                continue;
            end
            
            p1= p0 + [d*cos(a(j)+particles(i).p(3,step)); d*sin(a(j)+particles(i).p(3,step))];
            p_start = world2map(p0, params);
            p_end = world2map(p1, params);
            l = bresenham(p_start, p_end);
           
            if ~isempty(l)
                for k = 1:size(l,2)
                    particles(i).map.visit(l(2,k),l(1,k)) = particles(i).map.visit(l(2,k),l(1,k)) + 1;
                    particles(i).map.hit(l(2,k),l(1,k)) = particles(i).map.hit(l(2,k),l(1,k)) - params.hit_weight;
                end
            end
            
            if (d<params.usable_range(2)) && (d>params.usable_range(1))
                particles(i).map.hit(p_end(2),p_end(1)) = particles(i).map.hit(p_end(2),p_end(1)) + params.hit_weight;
                particles(i).map.visit(p_end(2),p_end(1))= particles(i).map.visit(p_end(2),p_end(1)) + 1;
            end
            
        end

        [index1, index2] = find(particles(i).map.hit);
        for k = 1 : length(index1)
            particles(i).map.occupy(index1(k), index2(k)) = exp(particles(i).map.hit(index1(k), index2(k))) / (1+exp(particles(i).map.hit(index1(k), index2(k))));
        end

    end
    % the end of register scan

end
% the end of all loop