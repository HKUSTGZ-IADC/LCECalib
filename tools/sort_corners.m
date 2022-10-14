function corners_out = sort_corners(corners_in)
  model = corners_in'\(-ones(size(corners_in,2),1));
  model = [model;1];
  model = model/norm(model(1:3));
  if model(4)<0
      model = -model;
  end
  center = mean(corners_in,2);
  
  vecs = corners_in - center;
  cos_thetas = vecs'*vecs(:,1);
  thetas = acosd(cos_thetas);
  for idx=1:size(corners_in,2)
     cross_product = cross(vecs(:,1),vecs(:,idx));
     direc = model(1:3)'*cross_product;
     if direc<0
        thetas(idx) = 360 - thetas(idx);
     end
  end
  
  [B,I] = sort(thetas);
  corners_out = corners_in(:,I);
  
end