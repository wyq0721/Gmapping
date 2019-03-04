%% Function description: compute the transformation from pose1 to pose2
%===============================================================================
% INPUT:
% @p1           pose 1
% @p2           pose 2
% OUTPUT:
% @delta        3x1 transformation vector from pose1 to pose2
% DATE:         2018/12/23 wyq
%===============================================================================

function delta = absolute_difference(p1,p2)% p1=p_new p2=p_old
	
    delta = p1-p2;
	delta(3) = atan2(sin(delta(3)), cos(delta(3)));%rot1+rot2
	s = sin(p2(3));
    c = cos(p2(3));
    
    %consruct 2D transform for t-pose from global map to t-1 local map
	delta = [ c*delta(1)+s*delta(2);...
             -s*delta(1)+c*delta(2);...
              delta(3)];
end