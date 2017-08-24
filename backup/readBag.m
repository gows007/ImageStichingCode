clear workspace
clear
close all
% mainDir  = fileparts(pwd);

globalFunctions =[fileparts(pwd) filesep 'ImageStichingCode_rosBag'];
bagName = [globalFunctions filesep '2017-07-25-17-19-58.bag'];
clear globalFunctions;
bag = rosbag(bagName);
disp(bag.AvailableTopics);
bagSelect = select(bag,'Topic','/feuerwerr_siso_demo/FeuerIRImage');
msgs = readMessages(bagSelect);
clear bagName bag bagSelect;
n = 1;

outputVideo = VideoWriter('ir_camera.avi');
outputVideo.FrameRate = 30;
open(outputVideo)

for i= 1:1:length(msgs)
    imgTemp = rot90(readImage(msgs{i}.Irimage),1);
    img1 = rot90(imgTemp, 1);    
    imgGS{n} = rgb2gray(img1);
    writeVideo(outputVideo,imgGS{n});
    n = n+1;
end

close(outputVideo);

  

% [CV_tforms,imageSize] = imageMosaic(imgGS(80:end));
close all

