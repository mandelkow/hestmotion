function Fig = hestmotionfig(P)
% hEstMotionFig(P):**1a++: Display P(M) for hEstMotion.m

DISP = 1;

%% Disp motion params:
if size(P,2)~=12,
    M = P;
    P = zeros(size(M,3),12);
    for ImgNo=1:size(M,3),
        P(ImgNo,:) = hspm_imatrix(M(:,:,ImgNo));
    end;
end

if DISP,
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

% %% Transform images for output
% if nargout > 2,
%     if Flag2D,
%         fprintf('\nTransform %u images... %5u',size(M,3),0);
%         for ImgNo=1:ImgSz(end),
%             Img(:,:,ImgNo) = warpAffine2(Img(:,:,ImgNo),M(:,:,ImgNo));
%             fprintf('\b\b\b\b\b%5u',ImgNo);
%         end;
%     else
%         fprintf('\nTransform %u volumes... %5u',size(M(:,:,:),3),0);
%         for ImgNo=1:ImgSz(end),
%             Img(:,:,:,ImgNo) = warpAffine3(Img(:,:,:,ImgNo),M(:,:,ImgNo));
%             fprintf('\b\b\b\b\b%5u',ImgNo);
%         end;
%     end;
%     fprintf(' \t DONE.\n');
% end;
