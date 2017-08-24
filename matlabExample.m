clearvars;
close all;

curDir   = pwd;
mainDir  = fileparts(curDir);

measurements =[mainDir filesep 'Measurements' filesep 'videoMat'];
addpath(genpath(measurements));
% load('matFiles/Horizontal_3.mat');
load('iphone.mat');

Img = I;
totalImg = length(Img);
clear I;
% totalImg = size(Img,1);
numImages = 12;
image_distance = floor(totalImg/numImages);
% image_distance = 100;
limit = image_distance*numImages;

n=1;
for i= 1:image_distance:limit    
    I{n} = Img{i};
    n = n+1;
end
grayImage = I{1};
points = detectSURFFeatures(grayImage);
[features, points] = extractFeatures(grayImage, points);

% Initialize all the transforms to the identity matrix. Note that the
% projective transform is used here because the building images are fairly
% close to the camera. Had the scene been captured from a further distance,
% an affine transform would suffice.
tforms(numImages) = projective2d(eye(3));

% Iterate over remaining image pairs
for n = 2:numImages
	if(n>numImages)
        break;
    end
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;


    % Detect and extract SURF features for I(n).
    grayImage = I{n};
    points = detectSURFFeatures(grayImage);
    [features, points] = extractFeatures(grayImage, points);

    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
%%
    if (size(indexPairs,1)>=4)   % minimum 4 points needed for projective transform
        matchedPoints = points(indexPairs(:,1), :);
        matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);
    else
%         imgGS = imgGS_old;
        points = pointsPrevious;
        features = featuresPrevious;
        I(n) = [];
%         tforms(n) = [];
        tforms(numImages) = [];
        n = n-1;
        numImages = numImages-1;
        disp('Image Deleted');
        continue;
    end
    %%
%     matchedPoints = points(indexPairs(:,1), :);
%     matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);

    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);

    % Compute T(1) * ... * T(n-1) * T(n)
    tforms(n).T = tforms(n-1).T * tforms(n).T;
end


imageSize = size(grayImage);  % all the images are the same size

% Compute the output limits  for each transform
for i = 1:numel(tforms)
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
end

avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);

Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)
    tforms(i).T = Tinv.T * tforms(i).T;
end

for i = 1:numel(tforms)
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(2)], [1 imageSize(1)]);
end

% Find the minimum and maximum output limits
xMin = min([1; xlim(:)]);
xMax = max([imageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([imageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(4*xMax - xMin);
height = round(4*yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width], 'like', grayImage);

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages

    

    % Transform I into the panorama.
    warpedImage = imwarp(I{i}, tforms(i), 'OutputView', panoramaView);

    % Generate a binary mask.
    mask = imwarp(true(size(I{i},1),size(I{i},2)), tforms(i), 'OutputView', panoramaView);

    % Overlay the warpedImage onto the panorama.
    figure(1);
    panorama = step(blender, panorama, warpedImage, mask);
    imshow(panorama);
    hold on;
    pause(0.3);
%     pause(0.5)
end
hold off;
% figure(200)
% imshow(panorama)
