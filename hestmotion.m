function [M,P,Img] = hestmotion(Img,varargin)
% hEstMotion **1b: Motion correction using estMotionMulti3() by <david.heeger@nyu.edu>.
%
% SYNTAX: [M,P,Img] = hestmotion(Img,[RefImg,]MultiScale,numIiters,Minitial,rotFlag,robustFlag,CB,SC)
%
% MultiScale = ? use multiscale algorithm
% numIters =
% Minitial =
% rotFlag = RIGID BODY only
% M(:,:,n) = Affine trafo matrix for img n
% P = Motion correction parameters
% Img = Corrected imgs
% SEE: estMotion2, estMotion3

% AUTH: HM, 04.2006, 2B: Implement DEFAULT.
% AUTH: HM, 04.2006, 1B1: BUGFIX: 2D RefImg for 2D correction.
% AUTH: HM, 02.2005, 1B.

DISP = 1;
% [RefImg,MultiScale,numIters,Minitial,rotFlag,robustFlag,CB,SC]
DEFAULT = [{Img(:,:,:,1)},{0},{1},{[]},{1},{0},{[]},{[]}];
if isempty(varargin) || length(varargin{1}) < 2, % No RefImg given.
    varargin = [{[]},varargin];
    ImgNo = 2;
else,
    ImgNo = 1;
end
tmp = find(~cellfun('isempty',varargin));
DEFAULT(tmp) = varargin(tmp);
varargin = DEFAULT;
clear DEFAULT
[RefImg,MultiScale,numIters,Minitial,rotFlag,robustFlag,CB,SC] = ...
    deal(varargin{:});
varargin(1:2) = [];
disp(cell2struct([{MultiScale},varargin],{'MultiScale','Iterations',...
    'Minitial','RigidFlag','RobustFlag','CB','SC'},2));
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
else,
    Flag2D = 0;
    M = zeros([4,4,ImgSz(end)]);
end;
if isempty(MultiScale) || ~MultiScale, % No Multi Scale approach
    FUN = strrep(FUN,'Multi','Iter');
end;
if numIters == 1,   % Only one iteration.
    FUN = strrep(FUN,'Iter','');
    varargin(1:2) = []; % Delete numIters and Minitial
end;
FUN = str2func(FUN);

if isempty(rotFlag) || rotFlag,
    MSG = strrep(MSG,'full affine','rigid-body');
end;
fprintf(MSG,ImgSz(end),0);
if ImgNo > 1,
    M(:,:,1) = eye(size(M,1));
end;
wstate = warning('off','hctranspose:complex');
for ImgNo=ImgNo:ImgSz(end),
    if Flag2D,
        M(:,:,ImgNo) = feval(FUN,RefImg,Img(:,:,ImgNo),varargin{:});
    else,
        M(:,:,ImgNo) = feval(FUN,RefImg,Img(:,:,:,ImgNo),varargin{:});
    end;
    if 0, %numIters > 1,
        varargin{2} = M(:,:,ImgNo); % = Minitial
    end;
    fprintf('\b\b\b\b\b%5u',ImgNo);
    if 0,
        subplot(2,1,1)
        plot(ImgNo,M(1,3,ImgNo),'r.');
        plot(ImgNo,M(2,3,ImgNo),'b.');
        subplot(2,1,1)
        plot(ImgNo,acos(M(1,1,ImgNo)),'m.');
        drawnow;
    end;
end;
fprintf(' \t DONE.\n');
warning(wstate);

%% Disp motion params:
if DISP,
    figure('name','hestmotion');
    
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
    
    for ImgNo=1:size(M,3),
        P(ImgNo,:) = hspm_imatrix(M(:,:,ImgNo));
    end;
    subplot(3,1,1)
    plot(P(:,1:3));
    ylabel('translation / voxel'); xlabel('image volumes');
    legend('X','Y','Z','location','best','orientation','horizontal');
    
    subplot(3,1,2)
    plot(180/pi*P(:,4:6));
    ylabel('rotation / deg'); %xlabel('image volumes');
    %legend('X','Y','Z','location','best','orientation','horizontal');
    
    subplot(6,1,5)
    plot(P(:,7:9));
    ylabel('scale factor'); %xlabel('image volumes');
    %legend('X','Y','Z','location','best','orientation','horizontal');
    
    subplot(6,1,6)
    plot(P(:,10:12));
    ylabel('shear'); %xlabel('image volumes');
    %legend('X','Y','Z','location','best','orientation','horizontal');
    
    set(findobj(gcf,'type','axes'),'FontSize',6,'FontWeight','normal');
end;

%%
if nargout > 2,
    if Flag2D,
        fprintf('\nTransform %u images... %5u',size(M,3),0);
        for ImgNo=1:ImgSz(end),
            Img(:,:,ImgNo) = warpAffine2(Img(:,:,ImgNo),M(:,:,ImgNo));
            fprintf('\b\b\b\b\b%5u',ImgNo);
        end;
    else,
        fprintf('\nTransform %u volumes... %5u',size(M(:,:,:),3),0);
        for ImgNo=1:ImgSz(end),
            Img(:,:,:,ImgNo) = warpAffine3(Img(:,:,:,ImgNo),M(:,:,ImgNo));
            fprintf('\b\b\b\b\b%5u',ImgNo);
        end;
    end;
    fprintf(' \t DONE.\n');
end;

return;

%% TEST:
% See hms_motioncorr.m
