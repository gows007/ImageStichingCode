
function [CV_tforms,imageSize]=imageMosaic(KeyFrame_matrix)
%% Estimate the geometric transform by using comupter vision
doPlot=[1,6];%doPlot=[1,10];
doPlotMatched=[1,6];

%% Preperation 

I=KeyFrame_matrix{1,1}; %50
numberOfFrames = size (KeyFrame_matrix,2);
% Use SURF as detector to detect features 
%     points1=detectHarrisFeatures(I,'MetricThreshold',10);
points1=detectMinEigenFeatures(I);
%Use SURF as descriptor to extract features
[features,points2]=extractFeatures(I,points1);%,'Blocksize',11);

% affine and projective geometric transformation
tforms(numberOfFrames)=projective2d(eye(3));

frameValid=false(numberOfFrames,1);
frameValid(1)=true;
  
%% Do the GT estimation by using computer vision  
for n=2:numberOfFrames

    pointsPrev=points2;
    featuresPrev=features;
    I_prev=I; 
    I=KeyFrame_matrix{1,n};
    % Use SURF as detector to detect features 
%     points1=detectHarrisFeatures(I,'MetricThreshold',10);
    points1=detectMinEigenFeatures(I);
    %Use SURF as descriptor to extract features
    [features,points2]=extractFeatures(I,points1);

    % Match the extracted features the Matchthreshold is chosen as 10 pixels
    indexPairs1=matchFeatures(features,featuresPrev,'Unique',true,'MatchThreshold',5,'MaxRatio',0.2);
    
    if (size(indexPairs1,1)>=4)   % minimum 4 points needed for projective transform
        frameValid(n) = true;
        matchedPoints1 = points2(indexPairs1(:,1),:);
        matchedPoints1Prev = pointsPrev(indexPairs1(:,2),:);
 
% show the matched features

        figure(100); ax = axes;
        if(matchedPoints1.length < 20)
            ptsLen = matchedPoints1.length;
        else
            ptsLen = 20;
        end
        showMatchedFeatures(I_prev,I,matchedPoints1(1:ptsLen),matchedPoints1Prev(1:ptsLen),'montage','Parent',ax);
        title(ax, 'First 20 Candidate point matches');
        legend(ax, 'Matched points 1','Matched points 2');
        
        try
            
% Estimate the GT by using the matched features. Here are two types of GT affine and projective

            [actTForm1,ip11,ip12]=estimateGeometricTransform(matchedPoints1,matchedPoints1Prev,...
                'projective','Confidence',99.9);%,'MaxDistance',2.5);
 
            %multiple the GT frame by frame          
            tforms(n).T=tforms(n-1).T*actTForm1.T;      %first image is refernce
            
            % show matched features
            if doPlotMatched(1)<=n&&doPlotMatched(2)>=n
                figure(200);
                pause(1)
                showMatchedFeatures(I,I_prev,ip11,ip12)
            end
        
        
        catch exc
            frameValid(n)=false;
            tforms(n).T=tforms(n-1).T;
            continue
        end
    else
        figure(2000+n);imshow(I);hold on;plot(points1.selectStrongest(50));
        frameValid(n)=false;
        tforms(n).T=tforms(n-1).T;
    end
end
 CV_tforms = tforms;

% % set middle of video as refernce image
% refTForm=tforms(floor(numberOfFrames/2));
% corrTForm=invert(refTForm);
% for n=1:numberOfFrames
%      tforms(n).T=corrTForm.T*tforms(n).T;
% 
% end

%% Initialize the panorama by calculate the min and max of the geometric transformation
imageSize = size(I);  % all the images are the same size
 
for i = 1:numel(tforms)
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
end

%Compute the average X limits for each transforms and find the image that is in the center.
%Only the X limits are used here because the scene is known to be horizontal. 
%If another set of images are used, both the X and Y limits may need to be used to find the center image.

avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);
Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)
    tforms(i).T = Tinv.T * tforms(i).T;
end


% Find the minimum and maximum output limits
xMin = min([1; xlim(:)]);
xMax = max([imageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([imageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width 1], 'like', im2double(I));

blender = vision.AlphaBlender('Operation', 'Blend', ...
    'OpacitySource', 'Input port');

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

totalMask=zeros(size(panoramaView));

% Thi is for creating a panorama video
% panoramaVW = VideoWriter('outWuerfelblick.avi');
% panoramaVW.FrameRate=10;
% open(panoramaVW);

%% Create the panorama by using the GT from computer vision
for i = 1:numberOfFrames
    if frameValid(i)
         I=KeyFrame_matrix{1,i};

        % Transform I into the panorama.
        warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
        % Create an mask for the overlay operation.
        warpedMask = imwarp(ones(size(I(:,:,1))), tforms(i), 'OutputView', panoramaView);

        % Clean up edge artifacts in the mask and convert to a binary image.
        warpedMask = warpedMask >= 1;

        totalMask=totalMask+warpedMask;
        totalOpacityMask=1./totalMask;

        opacityMask=totalOpacityMask.*warpedMask;
        opacityMask(isnan(opacityMask))=0;

        % Overlay the warpedImage onto the panorama.
        panorama = step(blender, panorama, im2double(warpedImage), opacityMask);

    end
    
    
    if i==numberOfFrames
        imwrite(panorama,'testOutp_panorama.png');
        figure
     
    end
end
imshow(panorama)
% close(panoramaVW);


% implay('outWuerfelblick.avi');
end


