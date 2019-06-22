function [Xd,Yd] = mls_rigid(p, q, H, W, max_core)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mls_rigid: A naive implementation of rigid moving least squares for
%            point handles.
%   Input:
%       p, q: correspondence points.
%       H, W: image height and image width.
%       max_core: user-defined number of CPU cores in the pool.
%   Output:
%       Xd,Yd: new position grids.
%   Ref:
%     S. Schaefer et al., "Image Deformation Using Moving Least Squares",
%     ACM Transactions on Graphics (TOG), 2006.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

no_ctp = size(p, 1); % number of control points

% Construct coordinate grids
rv = 1:1:H;
rc = 1:1:W;
[X, Y] = meshgrid( rc, rv );
Xd = zeros(H,W, 'double');
Yd = zeros(H,W, 'double');

% Startparallel pool
poolobj = gcp('nocreate'); % If no pool, do not create new one.
if isempty(poolobj)
    % Starting a pool with numCore threads
	numCore = min(max_core, feature('numCores'));
	parpool( numCore ); % utilize all physical cores
else
    poolsize = poolobj.NumWorkers;
end

% Clock starts
tic

%------------------------------------------------------------------------------
% Create weight matrices & Centroids
%------------------------------------------------------------------------------
parfor r=1:H
  fprintf('processing line = %d / %d\n', r, H);
  
  for c=1:W
    
    wm = zeros(1, no_ctp);
   
    for idx=1:no_ctp
      a = double(X(r,c)) - double(p(idx, 1));
      b = double(Y(r,c)) - double(p(idx, 2));
      
      % Weights
      if( a==0 && b==0 )
        wm(idx) = 10^6;
      else
        wm(idx) = 1/(a*a + b*b);
        
      end      
    end

    % Centroids
    s = sum(wm(:));
    pCenX = (wm * p(:,1)) / s; % p*  1x1
    pCenY = (wm * p(:,2)) / s;
    qCenX = (wm * q(:,1)) / s; % q*  1x1
    qCenY = (wm * q(:,2)) / s;

    pHatX = p(:,1) - pCenX;
    pHatY = p(:,2) - pCenY;
    qHatX = q(:,1) - qCenX;
    qHatY = q(:,2) - qCenY;

    pHat = [pHatX, pHatY];
    qHat = [qHatX, qHatY];
    qCen = [qCenX, qCenY];
    pCen = [pCenX, pCenY];
    
    pHatPen = [pHat(:,2)   , -pHat(:,1) ];  % -(pHat)#  , (x,y)# = (-y,x)
    qHatPen = [qHat(:,2)   , -qHat(:,1) ];
    
    a = 0.0;
    b = 0.0;
    fs = zeros(1,2, 'double');
    for idx=1:no_ctp
      a = wm(idx) * (qHat(idx,:)*(pHat(idx,:))')     + a;
      b = wm(idx) * (qHat(idx,:)*(-pHatPen(idx,:))') + b;
      fs = wm(idx) * ( (([X(r,c), Y(r,c)] - pCen) * [pHat(idx,:) ; pHatPen(idx,:)]) * [qHat(idx,:)', qHatPen(idx,:)'] ) + fs;
    end
    mur = sqrt( a*a + b*b );
    fs = fs / mur; % scale fs by mur before going to final scaling by len_vp & len_fs
    len_vp = sqrt( (X(r,c)-pCenX)^2 + (Y(r,c)-pCenY)^2 ) ; % |v - pCen|
    len_fs = sqrt( fs(1)^2 + fs(2)^2 );
    fs = len_vp*fs/len_fs + qCen;

    % Update coordinate
    Xd(r,c) = fs(1);
    Yd(r,c) = fs(2);

  end % col
end % row

% Clock stops
toc

end % mls_rigid()

