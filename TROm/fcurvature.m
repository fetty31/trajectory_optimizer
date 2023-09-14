function [L,R] = fcurvature(coord)
% Radius of curvature for 2D or 3D curve
%  [L,R] = curvature(coord)
%   coord:   2 or 3 column array of x, y (and possibly z) coordiates
%   L:   Cumulative arc length
%   R:   Radius of curvature

  N = size(coord,1);
  dims = size(coord,2);
  if dims == 2
    coord = [coord,zeros(N,1)];  % Do all calculations in 3D
  end
  L = zeros(N,1);
  R = NaN(N,1);
  for i = 2:N-1
    [R(i),~,~] = circumcenter(coord(i,:)',coord(i-1,:)',coord(i+1,:)');
    L(i) = L(i-1)+norm(coord(i,:)-coord(i-1,:));
  end
  i = N;
  L(i) = L(i-1)+norm(coord(i,:)-coord(i-1,:));
end

