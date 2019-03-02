function p=world2map(pnt,params)
    center=reshape(params.world_center,[2,1]);
    center=repmat(center,1,size(pnt,2));
    p=round((pnt+center)/params.grid_size);
end