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
%
% AUTHOR: Hendrik.Mandelkow@gmail.com

% AUTH: HM, 07.2012, 3B4: Add outputs Fig and Par to suppress display.
% AUTH: HM, 07.2012, 3B1: Input Minitial=0/1 to pass M for consec. images.
% AUTH: HM, 08.2008, 3B: Replace param. MultiScale with numIters= [1 2 3].
%                        Display P(0,:), in case of only 1 image.
% AUTH: HM, 04.2006, 2B: Implement DEFAULT.
% AUTH: HM, 04.2006, 1B1: BUGFIX: 2D RefImg for 2D correction.
% AUTH: HM, 02.2005, 1B.

%%
DISP = 0; % ***
VERB = 1;
% [RefImg,numIters,Minitial,rotFlag,robustFlag,CB,SC]
DEFAULT = [{Img(:,:,:,1)},{1},{1},{1},{0},{[]},{[]}];

%%
if isempty(varargin) || length(varargin{1}) < 2, % No RefImg given.
    varargin = [{[]},varargin];
    ImgNo = 2;
else
    ImgNo = 1;
end
tmp = find(~cellfun('isempty',varargin));
DEFAULT(tmp) = varargin(tmp); % Replace defaults by input.
varargin = DEFAULT;
clear DEFAULT % Why?
[RefImg,numIters,Minitial,rotFlag,robustFlag,CB,SC] = deal(varargin{:});
varargin(1) = []; % Remove RefImg.
% Minitial only serves as a flag henceforth.
if isempty(Minitial), Minitial = 1; % (default) pass M betw images
elseif numel(Minitial)==1, varargin{2} = [];
end
Par = cell2struct(varargin,...
    {'Iterations','Minitial','RigidFlag','RobustFlag','CB','SC'},2);
if nargout > 4, VERB = 0; end
if VERB,
    disp(Par);
%     disp(cell2struct(varargin,...
%         {'Iterations','Minitial','RigidFlag','RobustFlag','CB','SC'},2));
end
% if length(numIters) < 2,
%     disp(cell2struct(varargin,...
%         {'Iterations','Minitial','RigidFlag','RobustFlag','CB','SC'},2));
% else
%     disp(cell2struct(varargin,...
%         {'Iterations_MultiScale','Minitial','RigidFlag','RobustFlag','CB','SC'},2));
% end
% Remove trailing empty inputs from varargin.
while ~isempty(varargin) && isempty(varargin{end}),
    varargin(end) = [];
end;

ImgSz = size(Img(:,:,:,:));
ImgSz(3) = size(Img,3);     % Make sure we have 3 dims at least.
FUN = 'estMotionMulti3';
MSG = '\nEstimate %u full affine transformations in 3D... %5u';
if length(ImgSz) < 4,   % 2D registration
    RefImg = RefImg(:,:,1);
    FUN = strrep(FUN,'3','2');
    MSG = strrep(MSG,'3D','2D');
    M = zeros([3,3,ImgSz(end)]);
    Flag2D = 1;
else
    Flag2D = 0;
    M = zeros([4,4,ImgSz(end)]);
end;
if length(numIters) < 2, % No Multi Scale approach
    FUN = strrep(FUN,'Multi','Iter');
    if numIters == 1,   % Only one iteration.
        FUN = strrep(FUN,'Iter','');
        varargin(1:2) = []; % Delete numIters and Minitial
    end;
end
FUN = str2func(FUN);

if isempty(rotFlag) || rotFlag,
    MSG = strrep(MSG,'full affine','rigid-body');
end;
if VERB, fprintf(MSG,ImgSz(end),0); end
if ImgNo > 1,
    M(:,:,1) = eye(size(M,1));
end;
wstate = warning('off','hctranspose:complex');
for ImgNo=ImgNo:ImgSz(end),
% parfor ImgNo=ImgNo:ImgSz(end), % Minitial must be 0!
    if Flag2D,
        M(:,:,ImgNo) = feval(FUN,RefImg,Img(:,:,ImgNo),varargin{:});
    else
        M(:,:,ImgNo) = feval(FUN,RefImg,Img(:,:,:,ImgNo),varargin{:});
    end;
    if Minitial~=0,
        varargin{2} = M(:,:,ImgNo); % = Minitial
    end;
    if VERB, fprintf('\b\b\b\b\b%5u',ImgNo); end
    if 0,
        subplot(2,1,1)
        plot(ImgNo,M(1,3,ImgNo),'r.');
        plot(ImgNo,M(2,3,ImgNo),'b.');
        subplot(2,1,1)
        plot(ImgNo,acos(M(1,1,ImgNo)),'m.');
        drawnow;
    end;
end;
if VERB, fprintf(' \t DONE.\n'); end
warning(wstate);

%% Disp motion params:
if DISP || (nargout > 1),
    P = zeros(size(M,3),12);
    for ImgNo=1:size(M,3),
        P(ImgNo,:) = hspm_imatrix(M(:,:,ImgNo));
    end;
end
Fig = [];
if DISP || (nargout==0) || (nargout==4),
    Fig = figure('name','hestmotion');
    
%     subplot(2,1,1);% set(gca,'xlim',[1,size(Img,4)]);
%     plot(squeeze(M(1:3,end,:))');
%     ylabel('translation / voxel'); xlabel('image volumes');
%     legend('X','Y','Z','location','best','orientation','horizontal');
%     if rotFlag,
%         subplot(2,1,2);% set(gca,'xlim',[1,size(Img,4)]);
%         plot([acos(squeeze(M(1,1,:))),...
%             acos(squeeze(M(2,2,:))),...
%             acos(squeeze(M(3,3,:)))]);
%         ylabel('rotation / rad'); xlabel('image volumes');
%         legend('X','Y','Z','location','best','orientation','horizontal');
%     end;
    
    P = cat(1,[0 0 0 0 0 0 1 1 1 0 0 0],P); % Display 0 in case size(M,3)==1.
    tmp = [0:size(P,1)-1]';
    
    subplot(3,1,1)
    plot(tmp,P(:,1:3));
    ylabel('translation / voxel'); xlabel('image volumes');
    legend('X','Y','Z','location','best','orientation','horizontal');
    
    subplot(3,1,2)
    plot(tmp,180/pi*P(:,4:6));
    ylabel('rotation / deg'); %xlabel('image volumes');
    %legend('X','Y','Z','location','best','orientation','horizontal');
    
    subplot(6,1,5)
    plot(tmp,P(:,7:9));
    ylabel('scale factor'); %xlabel('image volumes');
    %legend('X','Y','Z','location','best','orientation','horizontal');
    
    subplot(6,1,6)
    plot(tmp,P(:,10:12));
    ylabel('shear'); %xlabel('image volumes');
    %legend('X','Y','Z','location','best','orientation','horizontal');
    
    set(findobj(gcf,'type','axes'),'FontSize',6,'FontWeight','normal');
    
    P(1,:) = []; % Remove extra.
end;

%% Transform images for output
if nargout > 2 && nargout < 5,
    if Flag2D,
        if VERB, fprintf('\nTransform %u images... %5u',size(M,3),0); end
        for ImgNo=1:ImgSz(end),
            Img(:,:,ImgNo) = warpAffine2(Img(:,:,ImgNo),M(:,:,ImgNo));
            if VERB, fprintf('\b\b\b\b\b%5u',ImgNo); end
        end;
    else
        if VERB, fprintf('\nTransform %u volumes... %5u',size(M(:,:,:),3),0); end
        for ImgNo=1:ImgSz(end),
            Img(:,:,:,ImgNo) = warpAffine3(Img(:,:,:,ImgNo),M(:,:,ImgNo));
            if VERB, fprintf('\b\b\b\b\b%5u',ImgNo); end
        end;
    end;
    if VERB, fprintf(' \t DONE.\n'); end
else
    Img = [];
end;

return;

%% TEST:
% See hms_motioncorr.m
