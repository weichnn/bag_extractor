% Undistorts the given image using a set of intrinsic parameters.
%
% Note that this code was taken from Jean-Yves Bouguet's excellent Camera
% Calibration Toolbox for matlab which can be found in its entirety here:
%   http://www.vision.caltech.edu/bouguetj/calib_doc/
%
% Args:
%   I - the distorted image, an HxW double matrix between 0 and 255.
%   fc - 2x1 vector, the focal length parameters.
%   cc - 2x1 vector, the camera center parameters.
%   kc - 5x1 vector, the distortion parameters.
%   alpha_c - the skew coefficient, a scalar. 
%
% Returns:
%   I2 - the undistorted image, an HxW double matrix between 0 and 255.
function I2 = undistort_depth(I, fc, cc, kc, alpha_c, noiseMask)
  KK_new = [fc(1) alpha_c*fc(1) cc(1);0 fc(2) cc(2) ; 0 0 1];
  [I2] = rect(I,eye(3),fc,cc,kc, alpha_c, KK_new, noiseMask);
end

function [Irec] = rect(I, R, f, c, k, alpha, KK_new, noiseMask)


  if nargin < 5,
     k = [0;0;0;0;0];
     if nargin < 4,
        c = [0;0];
        if nargin < 3,
           f = [1;1];
           if nargin < 2,
              R = eye(3);
              if nargin < 1,
                 error('ERROR: Need an image to rectify');
              end;
           end;
        end;
     end;
  end;


  if nargin < 7,
     if nargin < 6,
      KK_new = [f(1) 0 c(1);0 f(2) c(2);0 0 1];
     else
      KK_new = alpha; % the 6th argument is actually KK_new   
     end;
     alpha = 0;
  end;



  % Note: R is the motion of the points in space
  % So: X2 = R*X where X: coord in the old reference frame, X2: coord in the new ref frame.


  if ~exist('KK_new'),
     KK_new = [f(1) alpha*f(1) c(1);0 f(2) c(2);0 0 1];
  end;


  [nr,nc] = size(I);

  Irec = 255*ones(nr,nc);

  [mx,my] = meshgrid(1:nc, 1:nr);
  px = reshape(mx',nc*nr,1);
  py = reshape(my',nc*nr,1);

  rays = inv(KK_new)*[(px - 1)';(py - 1)';ones(1,length(px))];


  % Rotation: (or affine transformation):

  rays2 = R'*rays;

  x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];


  % Add distortion:
  xd = apply_distortion(x,k);


  % Reconvert in pixels:

  px2 = f(1)*(xd(1,:)+alpha*xd(2,:))+c(1);
  py2 = f(2)*xd(2,:)+c(2);


  % Interpolate between the closest pixels:

  px_0 = floor(px2);

  py_0 = floor(py2);

  good_points = find((px_0 >= 0) & (px_0 <= (nc-2)) & (py_0 >= 0) & (py_0 <= (nr-2)));
  
  px2 = px2(good_points);
  py2 = py2(good_points);
  px_0 = px_0(good_points);
  py_0 = py_0(good_points);

  alpha_x = px2 - px_0;
  alpha_y = py2 - py_0;

  a1 = (1 - alpha_y).*(1 - alpha_x);
  a2 = (1 - alpha_y).*alpha_x;
  a3 = alpha_y .* (1 - alpha_x);
  a4 = alpha_y .* alpha_x;

  ind_lu = px_0 * nr + py_0 + 1;
  ind_ru = (px_0 + 1) * nr + py_0 + 1;
  ind_ld = px_0 * nr + (py_0 + 1) + 1;
  ind_rd = (px_0 + 1) * nr + (py_0 + 1) + 1;

  ind_new = (px(good_points)-1)*nr + py(good_points);

  % Ignore coeffs when they are indexing into noise.
  a1 = a1 .* ~noiseMask(ind_lu);
  a2 = a2 .* ~noiseMask(ind_ru);
  a3 = a3 .* ~noiseMask(ind_ld);
  a4 = a4 .* ~noiseMask(ind_rd);
  
  s = a1 + a2 + a3 + a4;
  
  badPix = s == 0;
  
  a1 = a1 ./ s;
  a2 = a2 ./ s;
  a3 = a3 ./ s;
  a4 = a4 ./ s;
  
  a1(badPix) = 0;
  a2(badPix) = 0;
  a3(badPix) = 0;
  a4(badPix) = 0;

  Irec = zeros(nr, nc);
  Irec(ind_new) = a1 .* I(ind_lu) + a2 .* I(ind_ru) + a3 .* I(ind_ld) + a4 .* I(ind_rd);
end