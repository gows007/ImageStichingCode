%% Image Mosaicing
% Author        : Arun Kumar
% Email         : mail.drgkumar@gmail.com
% Version       : 1.0
% Date          : 6/6/2014

%% %clear workspace
clear;
clc;
close all;
%read Reference image and convert into single
rgb1= im2single(imread('r1.jpg'));
I1 = rgb2gray(rgb1);
%create mosaic background
sz= size(I1)+300;% Size of the mosaic
h=sz(1);w=sz(2);
%create a world coordinate system
outputView = imref2d([h,w]);
%affine matrix
xtform = eye(3);
% Warp the current image onto the mosaic image
%using 2D affine geometric transformation
mosaic = imwarp(rgb1, affine2d(xtform),'OutputView', outputView);
%read Target image and convert into single
rgb2= im2single(imread('t1.jpg'));
I2 = rgb2gray(rgb2);
%find surf features of reference and target image ,then find new 
%affine matrix
%Detect SURFFeatures in the reference image
points = detectSURFFeatures(I1);
%detectSURFFeatures returns information about SURF features detected 
%in the 2-D grayscale input image . The detectSURFFeatures function 
%implements the Speeded-Up Robust Features (SURF) algorithm 
%to find blob features
%Extract feature vectors, also known as descriptors, and their 
%corresponding locations
[featuresPrev, pointsPrev] = extractFeatures(I1,points);
%Detect SURFFeatures in the target image
points = detectSURFFeatures(I2);
%Extract feature vectors and their corresponding locations
[features, points] = extractFeatures(I2,points); 
% Match features computed from the refernce and the target images
indexPairs = matchFeatures(features, featuresPrev);  
% Find corresponding locations in the refernce and the target images
matchedPoints     = points(indexPairs(:, 1), :);
matchedPointsPrev = pointsPrev(indexPairs(:, 2), :);  
%compute a geometric transformation from the  corresponding locations
tform=estimateGeometricTransform(matchedPoints,matchedPointsPrev,'affine');
xtform = tform.T;
% Warp the current image onto the mosaic image
mosaicnew = imwarp(rgb2, affine2d(xtform), 'OutputView', outputView);
%create a object to overlay one image over another
halphablender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');
% Creat a mask which specifies the region of the target image.
% BY Applying geometric transformation to image
mask= imwarp(ones(size(I2)), affine2d(xtform), 'OutputView', outputView)>=1;
%overlays one image over another
mosaicfinal = step(halphablender, mosaic,mosaicnew, mask);
%show results
figure,imshow(rgb1,'initialmagnification','fit');
figure,imshow(rgb2,'initialmagnification','fit');
figure,imshow(mosaicfinal,'initialmagnification','fit');
