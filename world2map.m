%% Function description: compute score for every pose that depends on scan the consitence with current map 
%===============================================================================
% INPUT:
% @pnt              pose in world frame 
% @params           listing in the up one level
% OUTPUT:
% @p             	pose in map frame
% DATE:             2018/12/23 wyq
%===============================================================================

function p = world2map(pnt, params)

    center = reshape(params.world_center, [2,1]);
    center = repmat(center, 1, size(pnt,2));
    p = round((pnt+center) / params.grid_size);
    
end