clc
clear 
close all
LeftImages = imageSet(fullfile('F:\Downloads\ImageUs\image_00\data'));
RightImages = imageSet(fullfile('F:\Downloads\ImageUs\image_01\data'));
nimages = LeftImages.Count;
VP1 = VideoWriter('driving1bw.avi');
VP2 = VideoWriter('driving2bw.avi');

%CHANGE FRAME RATE UNSYNC
VP1.FrameRate=10;
VP2.FrameRate=10;

open(VP1);
open(VP2);

for frameIdx = 1:nimages
    ILO = imread(LeftImages.ImageLocation{frameIdx});
    writeVideo(VP1,ILO);
    
end
for frameIdx = 1:nimages
    IL1 = imread(LeftImages.ImageLocation{frameIdx});
    writeVideo(VP2,IL1);
    
end
 close(VP1);
 close(VP2);