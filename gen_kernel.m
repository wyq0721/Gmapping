function [move,step]=gen_kernel(prm,mp)
    kernel_size=prm.kernel_size/mp.grid_size;
    [x,y]=meshgrid(-kernel_size:kernel_size,-kernel_size:kernel_size);
    move=[x(:),y(:)]';
    dist=sum(move.^2,1);
    [dist,id]=sort(dist,'ascend');
    move=move(:,id);
    move(:,dist>kernel_size^2)=[];
    step=size(move,2);
end
