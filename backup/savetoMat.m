clear workspace;
clearvars;
close all;

bagName = 'Dummy_Horizontal_1.bag';
bag = rosbag(bagName);
disp(bag.AvailableTopics);
bagSelect = select(bag,'Topic','/feuerwerr_siso_demo/FeuerIRImage');
numMessages = bagSelect.NumMessages;

for i= 1:numMessages
    msgs = readMessages(bagSelect,i);
    imgTemp = rot90(readImage(msgs{1}.Irimage),1);
    img1 = rot90(imgTemp, 1);   
    imgGS = rgb2gray(img1);
    I{i} = imgGS;
end

save('DummyHorizontal_1.mat','I');