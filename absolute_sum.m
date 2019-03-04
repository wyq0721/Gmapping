%% Function description: apply the transformation on pose1
%===============================================================================
% INPUT:
% @p1           pose 1
% @p2           the transformation
% OUTPUT:
% @pose         new pose after transformation
% DATE:         2018/12/23 wyq
%===============================================================================

function pose = absolute_sum(p1,p2)% p1 = pose in particle at last timestap, p2 = odometry caculation resulsts
    
    s = sin(p1(3));
    c = cos(p1(3));
   
    % consruct 2D inverse-transform from particle map to world(global) map
	pose = p1 + [c*p2(1)-s*p2(2);...
                 s*p2(1)+c*p2(2);...
                 p2(3)] ;
end
