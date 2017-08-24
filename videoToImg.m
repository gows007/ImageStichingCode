clearvars;
v = VideoReader('File_000.mov');
numFrames = floor(v.Duration*v.FrameRate);

i = 1;
while hasFrame(v)
   img = readFrame(v);
   I{i} = rgb2gray(img);
   i = i+1;
end
