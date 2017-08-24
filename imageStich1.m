clear workspace;
clearvars;
close all;
% 
% bagName = 'Dummy_Horizontal_1.bag';
% bag = rosbag(bagName);
% disp(bag.AvailableTopics);
% bagSelect = select(bag,'Topic','/feuerwerr_siso_demo/FeuerIRImage');
% msgs = readMessages(bagSelect,1:33:2915);
% 
% image_distance = 1;
% % numberOfImg = floor(length(msgs)/image_distance);

% load('matFiles/Horizontal_1.mat');
load('iphone.mat');
Img = I;
clear I;
totalImg = size(Img,1);
numberOfImg = 5;
% image_distance = floor(totalImg/numberOfImg);
image_distance = 10;
limit = image_distance*numberOfImg;

n=1;
for i= 1:image_distance:limit    
    I{n} = Img{i};
    n = n+1;
end



MinQuality = 0.03;

imgGS = I{1};

%Detect features
points1 = detectHarrisFeatures(imgGS,'MinQuality', MinQuality);
% points1 = detectSURFFeatures(imgGS);

%Extract Features
[features,validPoints] = extractFeatures(imgGS,points1,'SURFSize',128);

%Plot Image
figure(100); imshow(imgGS); hold on;
plot(validPoints);



tforms(numberOfImg) = projective2d(eye(3));
%%Do the same for other images
for i= 2:numberOfImg
	if(i>numberOfImg)
        break;
    end
    %Saving details of previous images
    imgGS_old = imgGS;
    features_old = features;
    validPoints_old = validPoints;
    
    %Taking the next image
   
    imgGS = I{i};
    
    %Detect features
   points1 = detectHarrisFeatures(imgGS,'MinQuality', MinQuality);
%     points1 = detectSURFFeatures(imgGS);
    %Extract Features
    [features,validPoints] = extractFeatures(imgGS,points1,'SURFSize',128);
    
    %Harris
    numFeatures = features.NumFeatures;
    
    %SURF
%     numFeatures = size(features,1);

    if(numFeatures < 5)
        imgGS = imgGS_old;
        features = features_old;
        validPoints = validPoints_old;
        I(i) = [];
        tforms(i) = [];
        i = i-1;
        numberOfImg = numberOfImg-1;
        disp('Image Deleted');
        continue;
    end
    %Plot Image
    figure(100); imshow(imgGS); hold on;
    plot(validPoints);
    
    %Find features match between imgGS and imgGS_old
    indexPairs = matchFeatures(features,features_old,'MaxRatio',0.9,'Unique',true);
    
    if (size(indexPairs,1)>=4)   % minimum 4 points needed for projective transform
        matchedPoints=validPoints(indexPairs(:,1),:);
        matchedPoints_old=validPoints_old(indexPairs(:,2),:);
    else
        imgGS = imgGS_old;
        features = features_old;
        validPoints = validPoints_old;
        I(i) = [];
        tforms(i) = [];
        i = i-1;
        numberOfImg = numberOfImg-1;
        disp('Image Deleted');
        continue;
    end
    
%     matchedPoints = validPoints(indexPairs(:,1),:);
%     matchedPoints_old = validPoints_old(indexPairs(:,2),:);
    
    %Plot image
    figure(200+i); 
    showMatchedFeatures(imgGS,imgGS_old,matchedPoints,matchedPoints_old);
    
        % Estimate the transformation between I(n) and I(n-1).
    tforms(i) = estimateGeometricTransform(matchedPoints, matchedPoints_old,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 3000);

    % Compute T(1) * ... * T(n-1) * T(n)
    tforms(i).T = tforms(i-1).T * tforms(i).T;
    
end

%%Find Image Start
imageSize = size(imgGS);  % all the images are the same size

% Compute the output limits  for each transform
for i = 1:numel(tforms)
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
end

%Here only horizontal movement
avgXLim = mean(xlim, 2);
[~, idx] = sort(avgXLim);
centerIdx = floor((numel(tforms)+1)/2);
centerImageIdx = idx(centerIdx);

Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)
    tforms(i).T = Tinv.T * tforms(i).T;
end

%%Initialize Panorama
for i = 1:numel(tforms)
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
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
panorama = zeros([height width], 'like', I{1});

%%Create Panorama
blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numberOfImg

    
    % Transform I into the panorama.
    warpedImage = imwarp(I{i}, tforms(i), 'OutputView', panoramaView);

    % Generate a binary mask.
    mask = imwarp(true(size(I{i},1),size(I{i},2)), tforms(i), 'OutputView', panoramaView);

    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure(1);
imshow(panorama);
