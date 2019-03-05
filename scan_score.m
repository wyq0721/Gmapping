%% Function description: compute score for every pose that depends on scan the consitence with current map 
%===============================================================================
% INPUT:
% @p                robot pose  
% @r                new reading of scan
% @mp               map
% @params           listing in the up one level
% OUTPUT:
% @s             	score
% DATE:             2018/12/23 wyq
%===============================================================================

function s = scan_score(p, r, mp, params)

    a = linspace(-params.max_angle,params.max_angle,params.num_beams);
    s = 0;
    index1 = r<params.usable_range(2);
    index2 = r>params.usable_range(1);
    index = index1&index2;
    d = r(index)';
    a = a(index);
    p_end = repmat(p(1:2),1,sum(index)) + [d.*cos(a+p(3));d.*sin(a+p(3))];
    map_pnt = world2map(p_end,params);
     
    %  find hit move
    for i = 1:size(map_pnt,2)
        
        test_pnt = map_pnt(:,i);
        
        for j = 1:size(params.hit_kernel,2)
            
            pnt = test_pnt + params.hit_kernel(:,j);

            if mp.visit(pnt(2),pnt(1)) == 0
                continue;
            
            %find the closest 'busy'(occupied) cell until next is free
            elseif mp.occupy(pnt(2),pnt(1)) > params.occupied_threshold
               
                %trans to actual distance
                mu = sum(params.hit_kernel(:,j).^2)*params.grid_size^2;
                best_mu = mu;
                
                %the dist min, the score max
                s = s + exp(-1/params.gaussian_sigma*best_mu);
                
                %if get the 'occupied', jump the loop
                break;
            end
            
        end
        
    end
    
end
    


