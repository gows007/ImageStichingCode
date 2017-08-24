clear workspace
clear
bagName = '2017-07-25-17-19-58.bag';
bag = rosbag(bagName);
disp(bag.AvailableTopics);
bagSelect = select(bag,'Topic','/feuerwerr_siso_demo/FeuerIRImage');
msgs = readMessages(bagSelect);
clear bagName bag bagSelect;

imgTemp = rot90(readImage(msgs{1}.Irimage),1);
img1 = rot90(imgTemp, 1);
imgGS = rgb2gray(img1);

%Create world Co-ordinate Systems
sz= size(imgGS)+500;% Size of the mosaic
h=sz(1);w=sz(2);

%create a world coordinate system
outputView = imref2d([h,w]);
points1=detectSURFFeatures(imgGS,'MetricThreshold',10);
[features_new,points2_new]=extractFeatures(imgGS,points1.selectStrongest(40),'Method','SURF');
%affine matrix
xtform = eye(3);
n = 1;

for i= 1:5:length(msgs)
    imgTemp = rot90(readImage(msgs{i}.Irimage),1);
    imgGS_old = imgGS;
    % Warp the current image onto the mosaic image
    %using 2D affine geometric transformation
    mosaic = imwarp(imgGS_old, projective2d(xtform),'OutputView', outputView);

    img1 = rot90(imgTemp, 1);    
    imgGS = rgb2gray(img1);
    points1=detectSURFFeatures(imgGS,'MetricThreshold',10);
    featuresPrev = features_new;
    points2_prev = points2_new;
    [features_new,points2_new]=extractFeatures(imgGS,points1.selectStrongest(40),'Method','SURF');

    indexPairs=matchFeatures(features_new,featuresPrev,'Unique',true,'MatchThreshold',10);
  % Find corresponding locations in the refernce and the target images
    matchedPoints     = points2_new(indexPairs(:, 1), :);
    matchedPointsPrev = points2_prev(indexPairs(:, 2), :);  
    
    %compute a geometric transformation from the  corresponding locations
    tform=estimateGeometricTransform(matchedPoints,matchedPointsPrev,'projective');
    xtform = tform.T;
    % Warp the current image onto the mosaic image
    mosaicnew = imwarp(imgGS, projective2d(xtform), 'OutputView', outputView);
    %create a object to overlay one image over another
    halphablender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');
    % Creat a mask which specifies the region of the target image.
    % BY Applying geometric transformation to image
    mask= imwarp(ones(size(imgGS)), projective2d(xtform), 'OutputView', outputView)>=1;
    %overlays one image over another
    mosaicfinal = step(halphablender, mosaic,mosaicnew, mask);
%     figure,imshow(imgGS_old,'initialmagnification','fit');
%     figure,imshow(imgGS,'initialmagnification','fit');
    figure,imshow(mosaicfinal,'initialmagnification','fit');
%     figure,imshow(mosaicfinal,'initialmagnification','fit');
%     imshow(imgGS); hold on;
%     plot(points2);
%     pause(1);
    n = n+1;
end