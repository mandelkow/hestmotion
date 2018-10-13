### hestmotion.m : Matlab re-wrapper for an MRI image/volume coregistration method by [David Heeger (NYU)](http://www.cns.nyu.edu/heegerlab/?page=home).
My main addition is the iteration over MRI volumes in a 4D dataset for the purpose of fMRI motion correction.

The original code was forked from: http://www.cns.nyu.edu/heegerlab/content/software/imgregsoft/registration.tar

Thanks for sharing!

-*Hendrik Mandelkow*

### Usage: ...just look at hestmotion.m

```matlab
function [M,P,Img,Fig,Par] = hestmotion(Img,varargin)
% hEstMotion:**3B5++: Motion correction for 2D/3D image/volumes (affine trafo)...
%                  using estMotionMulti3() by <david.heeger@nyu.edu>.
%
% SYNTAX: [M,P,Img,Fig] = hestmotion(Img,[RefImg,]numIiters,Minitial,rotFlag,robustFlag,CB,SC)
% SYNTAX: [M,P,~,~,Par] = hestmotion(...) % No text output, Img and Fig
%
% numIters = # of iterations default= 1. If vector e.g. [2 3 4],...
%            use *multiscale* algorithm with 4 iterations at 1/4 size, 3
%            iter. at 1/2 size and 2 iter. at full size.
% Minitial = starting point for optimisation (passed between iterations)
% Minitial = 0 -> don't pass Minitial between consecutive images.
% rotFlag = RIGID BODY motion only (no shearing and scaling)
% M(:,:,n) = Affine trafo matrix for img n
% P(n,:) = 12 Motion correction parameters (computed from M(:,:,n))
%          3x translation (XYZ), 3x rotation, 3x scaling, 3x shearing
% Img = Corrected images. To view 2D timeseries side by side use:
%       implay(hdatrescale(cat(2,dimg,cimg),[0 1]))  
%
% NOTE: Apparently, NaNs can be used to mask the reference (or input) image
% to achieve exclusion of data from computations.
%
% NOTE: The coordinate origin and center of rotation is top left corner!
%
% NOTE: Large difference in dynamic range between images compared are
%       disadvantageous. Normalisation may improve results. Also, try
%       multiscale (initial downsampling) for large images.
%
% SEE: estMotion2, estMotion3
```
