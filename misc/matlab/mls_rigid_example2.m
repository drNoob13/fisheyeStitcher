%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: Tuan Ho
% Function: examples of image deformation with rigid moving least squares
% Description:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all;

method = 'MLS Rigid';

%--------------------------------------------------------------------------
% Select Control Points
%--------------------------------------------------------------------------

% 0533_Cam1
img = imread('input/test_image.jpg'); % Used for manual ct pts select in PT-GUI

name = 'Test Image';

[H, W] = size( img(:,:,1) );

load('input/Correspondence_points_for_MLS');
p  = movingPoints;
q  = fixedPoints;

% Visualize the control points
figure('Name', 'Visualization - Left Ring (REF)');
imshow( uint8(img) );
hold on;
scatter( [p(:,1)], [p(:,2)], 'square', 'filled', 'y');
hold on;
scatter( [q(:,1)], [q(:,2)], 'square', 'filled', 'g');
hold off;
display('Press any key to continue');
pause
% 
p = p;
q = q;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--------------------------------------------------------------------------
% Calculating Weights for Each Control Points
%--------------------------------------------------------------------------
[H, W] = size( img(:,:,1) ); % RGB img

% Close pool
if ~isempty(gcp('nocreate'))
    delete(gcp('nocreate'));
end

nc = feature('numCores');
max_core = nc;
if( nc > 2 )
    max_core = max_core - 2;
end

[Xd, Yd] = mls_rigid( p, q, H, W, max_core );

% Saving Grids for offline use
save('XY_MLS_Grid_example2', 'Xd', 'Yd');

% If you want to write 'Xd', 'Yd' to OpenCV-readable yml format, load the 
% 'XY_MLS_Grid_example.mat' and use a matlab2opencv package such as:
% https://gist.github.com/dangkhoasdc/991b6913a43a2a726743

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--------------------------------------------------------------------------
% Deformation
%--------------------------------------------------------------------------
% Construct coordinate grids
rv = 1:1:H;
rc = 1:1:W;
[X, Y] = meshgrid( rc, rv );
%
I_deform = zeros(size(img));
I_deform(:,:,1) = interp2( double(X), double(Y), double(img(:,:,1)), Xd, Yd, 'linear');
I_deform(:,:,2) = interp2( double(X), double(Y), double(img(:,:,2)), Xd, Yd, 'linear');
I_deform(:,:,3) = interp2( double(X), double(Y), double(img(:,:,3)), Xd, Yd, 'linear');

% Visualize
figure(); title('MLS Similarity');
subplot(1,2,1); 
imshow(img); hold on; scatter( [q(:,1)], [q(:,2)], 'square', 'filled', 'g');
title([name ' (orginal)']);
hold off;
subplot(1,2,2);
imshow(I_deform); hold on; scatter( [p(:,1)], [p(:,2)], 'square', 'filled', 'y');
subplot(1,2,2); 
imshow(uint8(I_deform)); hold on; scatter( [p(:,1)], [p(:,2)], 'square', 'filled', 'y');
title([name ' (deformed by ' method ')']);


% Close pool
if ~isempty(gcp('nocreate'))
    delete(gcp('nocreate'));
end

