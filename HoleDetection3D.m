function t = HoleDetection3D(p, L, t)

%% 3D holes detection algorithm
% This work extend the 2D holes detection strategy proposed by Gypaets 
% (2016.6.18) to 3D situation. Here, we propose the conectivity length of a 
% local manifold to replace the area of polygon to detect the irragular 
% triangular mesh patches which should be the holes. It achieves pretty good 
% performance in finding holes in 3D situation.
% Example of the conectivity length of a local manifold (p1p2p3p4):
%
%  p1 ----- p2
%  \       /   \
%   \     /     \
%    \   /       \
%     p3 ------- p4
%
% The connectivity length of above manifold is calculated as follows,
% Conlen(p3) = length(p1-p3) + length(p1-p2) + length(p2-p3) + length(p2-p4)
% + length(p3-p4).
% 
%
% Author: Chong WU, Department of Electronic Engineering,
% City University of Hong Kong, Hong Kong SAR, China.
% Released Date: 2018.10.16
% Inspired by the work of Gypaets, 2016.6.18, and URL is 
% "https://ww2.mathworks.cn/matlabcentral/fileexchange/57739-gypaets-findtheholes."
%
%
% If you have found any bugs, have any suggestions or problems, please contact me at
% Email: imroxaswc@gmail.com

%% Syntax: 
% Input: p is a N-by-3 point clouds data, L is a thredshold (its value
% usually takes 2 to 3), and t is a raw triangular mesh of p which needs to detect 
% and remove the holes.
% Output: t is a new triangular mesh after holes detection and elimination.

nV = length(p); % Number of points/vertices
nM = length(t); % Number of manifolds in triangulation

% Manifolds connectivity length
Mlength = sqrt((p(t(:, 1), 1) - p(t(:, 2), 1)).^2 + ...
    (p(t(:, 1), 2) - p(t(:,2), 2)).^2 + ...
    (p(t(:, 1), 3) - p(t(:,2), 3)).^2) + ...
    sqrt((p(t(:, 2), 1) - p(t(:, 3), 1)).^2 + ...
    (p(t(:, 2), 2) - p(t(:, 3), 2)).^2 + ...
    (p(t(:, 2), 3) - p(t(:, 3), 3)).^2) +...
    sqrt((p(t(:, 3), 1) - p(t(:, 1), 1)).^2 + ...
    (p(t(:, 3), 2) - p(t(:, 1), 2)).^2 + ...
    (p(t(:, 3), 3) - p(t(:, 1), 3)).^2);

% Vertex-Manifold matrix
mVerMan = sparse(t(:), repmat(1:nM, [1 3]), 1, nV, nM);
mVPL = bsxfun(@times, mVerMan, Mlength')';

% Maximum and minimum manifold connectivity length at each point
maxL = max(mVPL);
minL = max(spfun(@(x) x.^-1, mVPL)).^-1;

% Ratio between maximum and minimum manifold connectivity length 
% at each point

lrRat = maxL./minL;

% Select points with connectivity length ratio lrRat greater 
% than the threshold
borderPoints = find(lrRat > L)';

% Sparse array with border points indices
borderPointsS = sparse(1, borderPoints, 1, 1, nV);

% Number of border points in every manifold
bordVert = borderPointsS*mVerMan;

% Get index of manifolds with three border points
detPol = find(bordVert == 3);

% Remove manifolds connected to three border points
t(detPol, :) = [];

end
