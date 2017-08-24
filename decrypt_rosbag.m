function [] = decrypt_rosbag(filename,filename_q,saveImages,saveMat)
%------------------------- Description -----------------------------------%
% This function decrypts a ROSbag (.bag-File) to use it in MATLAB. 
%--------------------------- Input ---------------------------------------%
% filename........filename of rosbag
% filename_q......filename of Qualisys data
% saveImages......flag to switch on/off saving of IR-frames
% saveMat.........flag to switch on/off saving of measurement structure

%% example:
% decrypt_rosbag('Messung1',[pwd '/Qualisys/Messung1'],false,true)

%% 1) load rosbag
bag = rosbag([filename '.bag']);

% add rostopics which should be extracted

% !!! changed because of different message /feuerwerr_siso_demo/FeuerIRImage
bagSelect.IR_frames = select(bag,'Topic', '/feuerwerr_siso_demo/FeuerIRImage');
bagSelect.RadarRaw = select(bag,'Topic','/feuerwerr_siso_demo/FeuerRadarRaw');
bagSelect.Time = select(bag,'Topic','/feuerwerr_siso_demo/FeuerTime_IR');

nTopics = length(fieldnames(bagSelect));
topicNames = fieldnames(bagSelect);

%% 2) decript rosbag to MATLAB struct with timestamps
for j = 1:nTopics
    for k = 1:bagSelect.(topicNames{j}).NumMessages
        % read actual ros message
        tmp_msg = readMessages(bagSelect.(topicNames{j}),k);

        % RADAR raw measurements (imag, real)
        if strcmp(topicNames{j},'RadarRaw')
            % transform data                     
            DataStore.(topicNames{j}).imag{k} = reshape(tmp_msg{1}.IQmsg.Data(1:160000),1000,160);
            DataStore.(topicNames{j}).real{k} = reshape(tmp_msg{1}.IQmsg.Data(160001:320000),1000,160);

            % get time stamps
            DataStore.(topicNames{j}).t(k) = seconds(tmp_msg{1}.Header.Stamp);
        end

        % IR-camera raw frames
        if strcmp(topicNames{j},'IR_frames')
            % transform data 
            DataStore.(topicNames{j}).image{k} = flip(flip(tmp_msg{1}.Irimage.readImage),2);

            if saveImages == true
                if ~exist([filename '_PNG'], 'dir')
                    mkdir([filename '_PNG']);
                end
                imwrite(DataStore.(topicNames{j}).image{k},...
                       [pwd filesep filename '_PNG' filesep 'image', num2str(k), '.png']);
            end

            % get time stamps
            % !!! changed because of different message /feuerwerr_siso_demo/FeuerIRImage
            DataStore.(topicNames{j}).t(k) = seconds(tmp_msg{1}.Irimage.Header.Stamp);
        end

        % Qualysis time based on IMU trigger
        if strcmp(topicNames{j},'Time')
            % transform data
            DataStore.(topicNames{j}).time(k) = tmp_msg{1}.TimeRef.Sec +...
                                             tmp_msg{1}.TimeRef.Nsec*10^-9;
        end
    end
end
%% 3) load and get Qualisys position / attitude data
if ~isempty(filename_q)
    T_Qualisys = 1/20;
    delta_idx = round(DataStore.Time.time(1) * 1/T_Qualisys);

    Qualisys = load([filename_q '.mat']);
    nMeas = fieldnames(Qualisys);
    DataStore.Qualisys.Pos = squeeze(Qualisys.(nMeas{1}).RigidBodies.Positions(end,:,delta_idx:end))';
    DataStore.Qualisys.Pos = (DataStore.Qualisys.Pos - DataStore.Qualisys.Pos(1,:))./1000;
    DataStore.Qualisys.Att = squeeze(Qualisys.(nMeas{1}).RigidBodies.RPYs(end,:,delta_idx:end))';
end
%% 4) set timestamps of rosbag to zero
DataStore.IR_frames.t = DataStore.IR_frames.t - DataStore.IR_frames.t(1);
DataStore.RadarRaw.t = DataStore.RadarRaw.t - DataStore.RadarRaw.t(1);
DataStore.Time.time = DataStore.Time.time - DataStore.Time.time(1);

% !!! added because of doubled frequence of FeuerTimeIR on IMU
DataStore.Time.time = DataStore.Time.time([1 2:2:size(DataStore.Time.time,2)]);

%% 5) save MATLAB struct to .mat file
if saveMat == true
    save([filename '.mat'],'DataStore','-v7.3');
end
end
