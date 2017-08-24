function [Radar,IMU] = SensorTS_sync(IR_TS,DataStore)
%------------------------- Description -----------------------------------%
% This function synchronizes the actual IR frame with the IMU and Radar
% data based on closest timestamps in the past.
%--------------------------- Input ---------------------------------------%
% IR..............actual image with timestamp [timestamp]
% DataStore.......data structure of measurement file [IR Radar Time IMU]
%--------------------------- Output --------------------------------------%
% Radar...........actual Radar data structure [idx, delta, real, imag]
% IMU.............actual IMU data structure [idx, delta, Pos, Att]

%% 1) find closest Radar / IMU data based on actual IR timestamp

% Radar
Radar.idx = find(DataStore.RadarRaw.t <= IR_TS,1,'last');
Radar.delta = DataStore.RadarRaw.t(Radar.idx)-IR_TS;
Radar.real = DataStore.RadarRaw.real(Radar.idx);
Radar.imag = DataStore.RadarRaw.imag(Radar.idx);

% IMU
IMU.idx = find(DataStore.Time.time <= IR_TS,1,'last');
IMU.delta = DataStore.Time.time(IMU.idx)-IR_TS;
IMU.Pos = DataStore.Qualisys.Pos(IMU.idx,:);
IMU.Att = DataStore.Qualisys.Att(IMU.idx,:);
end