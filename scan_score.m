function s=scan_score(p,r,mp,params)
% p@robot_pose
% r@new_reading scan
% mp@map

    %% explanation
    % score depends on scan the consitence with current map
%     load workspace.mat;
    a=linspace(-params.max_angle,params.max_angle,params.num_beams);
    s=0;% for every pose
    l = 0 ;
    index1 = r<params.usable_range(2);
    index2 = r>params.usable_range(1);
    index = index1&index2;
    d = r(index)';
    a = a(index);
    p_end = repmat(p(1:2),1,sum(index)) + [d.*cos(a+p(3));d.*sin(a+p(3))];
    map_pnt=world2map(p_end,params);
     
    %  find hit move
    for i = 1:size(map_pnt,2)
        test_pnt = map_pnt(:,i);
        for j=1:size(params.hit_kernel,2)
            pnt=test_pnt+params.hit_kernel(:,j);

            if mp.visit(pnt(2),pnt(1))==0
                continue;
            elseif mp.occupy(pnt(2),pnt(1))>0.8%find the closest 'busy'(occupied) cell until next is free
                mu=sum(params.hit_kernel(:,j).^2)*params.grid_size^2;%trans to actual distance
                best_mu=mu;
                s=s+exp(-1/params.gaussian_sigma*best_mu);%the dist min, the score max
                break;%if get the 'odd' , jump the loop
            end
        end
    end
    
    


