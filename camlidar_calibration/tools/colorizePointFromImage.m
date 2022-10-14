function cloud_rgb = colorizePointFromImage(tf, K, pts, img)
  Pcam = K * (tf(1:3, 1:3) * pts(1:3, :) + tf(1:3, 4));
  ucam = Pcam(1:2, :) ./ Pcam(3, :);
  height = size(img, 1);
  width = size(img, 2); 
  if (size(img, 3) == 1)
    img_rgb = uint8(zeros(size(img, 1), size(img, 2), 3));
    img_rgb(:, :, 1) = img;
    img_rgb(:, :, 2) = img;
    img_rgb(:, :, 3) = img;
  else
    img_rgb = img;
  end  
  rgb = uint8(ones(size(img_rgb, 3), size(pts, 2)) * 255);
  
  for idx = 1: size(ucam, 2)
    uv = ucam(1:2, idx);
    if (uv(1) < 1 || uv(1) > width || uv(2) < 1 || uv(2) > height)
      continue;
    end
    if (Pcam(3, idx)) <= 0 
      continue;
    end
    u = floor(uv(1));
    v = floor(uv(2));
    color_int = ...
      (u + 1 - uv(1)) * (v + 1 - uv(2)) * vec(single(img_rgb(v, u, :))) + ...
      (uv(1) - u) * (v + 1 - uv(2)) * vec(single(img_rgb(v, u + 1, :))) + ...
      (u + 1 - uv(1)) * (uv(2) - v) * vec(single(img_rgb(v + 1, u, :))) + ...
      (uv(1) - u) * (uv(2) - v) * vec(single(img_rgb(v + 1, u + 1, :)));  
    rgb(:, idx) = uint8(color_int);   
  end
  cloud_rgb = pointCloud(pts', 'Color', rgb');
end