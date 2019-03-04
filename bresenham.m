%% Function description: Bresenham line algorithm (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
%===============================================================================
% INPUT:
% @start_pnt    start point of the line
% @end_pnt      end point of the line
% OUTPUT:
% @l            points on line 
% DATE:         2018/12/23 wyq
%===============================================================================

function l = bresenham(start_pnt, end_pnt)
    
    x0 = start_pnt(1);
    x1 = end_pnt(1);
    y0 = start_pnt(2);
    y1 = end_pnt(2);
    
    dx = x1-x0;
    dy = y1-y0;
    
    if abs(dx) >= abs(dy)
        da = abs(dx);
        db = abs(dy);
        err = abs(db)/2;
        dira = sign(dx);
        dirb = sign(dy);        
    else
        da = abs(dy);
        db = abs(dx);
        err = abs(db)/2;
        dira = sign(dx);
        dirb = sign(dy);
    end
    
    l = zeros(2,3000);
    j = 0;
    cnt = 1;
    for i = 0:da-1
        err = err + db;
        if err >= da
            j = j + 1;
            err = err - da;
        end 
       
        if abs(dx) >= abs(dy)   
            l(:, cnt) = start_pnt + [dira*i; dirb*j];
            cnt = cnt + 1;
        else
            l(:, cnt) = start_pnt + [dira*j; dirb*i];
            cnt = cnt + 1;
        end
    end  
    l(:, cnt:end) = [];    
end
