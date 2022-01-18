%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: Tuan Ho
% Function: Moving Least Squares (Rigid)
% Description:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all;

method = 'MLS Rigid';


%--------------------------------------------------------------------------
% Select Control Points
%--------------------------------------------------------------------------

% Mona Lisa:
%    cropped from: https://en.wikipedia.org/wiki/File:Mona_Lisa.jpg
img = imread('input/Mona_Lisa_crop.jpg');
name = 'Mona Lisa';
%
rest_pts   = [[219,276];[147,282];[158,254];[193,255];[179,338];[274,291];[121,286];[107,214];[146,185];[227,185];[284,217];[287,121];[113,125]]; % fixed points

% Smile
% mov_pts = [[229, 266];[141,267];[158,254];[193,255];[179,338];[274,291];[121,286];[107,214];[146,185];[227,185];[284,217];[287,121];[113,125]]; % moving

% Mixed feeling
% mov_pts = [[228,268];[147,282];[158,254];[193,255];[179,338];[274,291];[121,286];[107,214];[146,185];[227,185];[284,217];[287,121];[113,125]]; % moving

% Sad
mov_pts = [[225, 299];[143,297];[158,254];[193,255];[179,338];[274,291];[121,286];[107,214];[146,185];[227,185];[284,217];[287,121];[113,125]]; % moving

p = mov_pts;
q = rest_pts;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--------------------------------------------------------------------------
% Calculating Weights for Each Control Points
%--------------------------------------------------------------------------
[H, W] = size( img(:,:,1) ); % RGB img

max_core = feature('numCores');

% MLS
[Xd, Yd] = mls_rigid( p, q, H, W, max_core );

% Saving Grids for offline use
save('XY_MLS_Grid_example1', 'Xd', 'Yd');

% If you want to write 'Xd', 'Yd' to OpenCV-readable yml format, use:
% a matlab2opencv package:
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
imshow(img); hold on; scatter( [q(:,1)], [q(:,2)], 'square', 'filled', 'y');
title([name ' (orginal)']);
hold off;
subplot(1,2,2); 
imshow(I_deform); hold on; scatter( [p(:,1)], [p(:,2)], 'square', 'filled', 'y');
subplot(1,2,2); 
imshow(uint8(I_deform)); hold on; scatter( [p(:,1)], [p(:,2)], 'square', 'filled', 'y');
title([name ' (deformed by ' method ')']);


% Close pool
% if ~isempty(gcp('nocreate'))
%     delete(gcp('nocreate'));
% end

