%% Function description: compute Gaussian noise 
%===============================================================================
% INPUT:
% @sigma            variance of Gaussian distribution
% OUTPUT:
% @noise            noise for initial guess
% DATE:             2018/12/23 wyq
%===============================================================================

function noise = sample_gaussian(sigma)

    if (sigma == 0)
        noise = 0;
        return;
    end
    
    w = 2;
    
    while(w > 1.0 || w == 0.0)
        r = rand(1); 
        x1 = 2.0*r - 1.0;% -1~1
        r = rand(1);
        x2 = 2.0*r - 1.0;% -1~1
        w = x1*x1 + x2*x2; % 0~1
    end
    
  	noise = sigma*x2*sqrt(-2.0*log(w)/w);
    
end