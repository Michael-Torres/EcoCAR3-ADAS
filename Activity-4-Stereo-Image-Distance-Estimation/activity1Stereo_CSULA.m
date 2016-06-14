
clc
clear
close all
% open file
%% Load kitti calibration parameters
loadedStereoParams = loadCalibrationCamToCam();

%% CSULA_loadKitti
% This part loads the kitti stereo parameters and formats them for Matlab

%convert cell to vector
%extract intrinsic matrices
K_02=loadedStereoParams.K{1,3}';
K_03=loadedStereoParams.K{1,4}';
%extract distortion coeffs
D_02=loadedStereoParams.D{1,3};
radialCoeffMask=logical([1,1,0,0,1]);
%extract radial and tangential distorion coeffs
RD_02=D_02(radialCoeffMask);
TD_02=D_02(~radialCoeffMask);

D_03=loadedStereoParams.D{1,4};
RD_03=D_03(radialCoeffMask);
TD_03=D_03(~radialCoeffMask);
%%
%create camera parameter obj with T and R
cameraParams1_3 = cameraParameters('IntrinsicMatrix',K_02,...
    'RadialDistortion',RD_02,'TangentialDistortion',TD_02);
cameraParams1_4 = cameraParameters('IntrinsicMatrix',K_03,...
    'RadialDistortion',RD_03,'TangentialDistortion',TD_03);

%%
%extract rotation and translation
R_03=loadedStereoParams.R{1,4}';
T_03=loadedStereoParams.T{1,4}';
%%
%create Matlab stereo obj
stereoParams_kitti=stereoParameters(cameraParams1_3,cameraParams1_4,R_03,T_03);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CSULA distance detector with stereo cameras
% This part of the script detects the distance

%load stereo videos
LV = vision.VideoFileReader('driving1_unsync.avi');
RV = vision.VideoFileReader('driving2_unsync.avi');

%%Make Video
VR = vision.VideoFileWriter('driving.mp4','FrameRate',LV.info.VideoFrameRate,'FileFormat','MPEG4');
VR.FrameRate=10;

%%show Video
VP = vision.VideoPlayer;

%% crop video mid-section
tf = 1/4;
bf = 3/4;

%% copy stereoParams for local calculations
stereoParams= stereoParams_kitti;

%%
i=1;
while(~isDone(LV))
    
    %%Get Current frame
    ILO = step(LV);
    IRO = step(RV);
%%
%%    %% Extract image
    %% Extract the middle of the image
    [nr,nc,temp] = size(ILO);
    IL = ILO(tf*nr:bf*nr,:,:);
    IR = IRO(tf*nr:bf*nr,:,:);
    % View the extracted image
    % figure; 
    % imshowpair(IL,IR,'montage'); 
    % title('Extracted Portion of Original Images');

    %% Rectify the images.
    [JL, JR] = rectifyStereoImages(IL, IR, stereoParams);

    %% Generate Disparity Map
    % Create the disparity map 
    disparityRange = [0 64];
    disparityMap = disparity(rgb2gray(JL),rgb2gray(JR),'DisparityRange',disparityRange);
    % View the disparity map
    %figure; 
    %imshow(disparityMap,disparityRange); 
    %title('Disparity Map'); 
    %colormap('jet'); 
    %colorbar;

    %% Reconstruct Point Cloud
    % create an empty stereo parameters object
    ptCloud = reconstructScene(disparityMap, stereoParams);
    % Convert from millimeters to meters.
    ptCloud = ptCloud/1000;
    % Limit the range of Z and X for display.
    thresholds=[-5 5; -5 10; 0 30];  
    
    % unpack thresholds
    xl = thresholds(1,1);
    xu = thresholds(1,2);
    yl = thresholds(2,1);
    yu = thresholds(2,2);
    zl = thresholds(3,1);
    zu = thresholds(3,2);

    % unpackage point cloud
    x = ptCloud(:,:,1);
    y = ptCloud(:,:,2);
    z = ptCloud(:,:,3);
    % threshold point cloud
    x(x < xl | x > xu) = NaN; 
    y(y < yl | y > yu) = NaN;
    z(z < zl | z > zu) = zu;
    % package point cloud
    ptCloud(:,:,1) = x;
    ptCloud(:,:,2) = y;
    ptCloud(:,:,3) = z;

    % View point cloud
%      figure
%      pcshow(ptCloud, JL)
%      ha = gca;
%      ha.CameraViewAngle = 5;
%      ha.CameraUpVector = [0 -1 0];
%      ha.CameraPosition = [-15 -10 -110];
%      ha.CameraTarget = [0 -2 15];
%      xlabel('X');
%      ylabel('Y');
%      zlabel('Z');
%      title('Point Cloud');
   
    %% Identify the depth of the vehicle in front
    % show rectified image which corresponds to the point cloud
    COD = vision.CascadeObjectDetector('CarDetector.xml');
    bboxes = step(COD,JL);

%     bboxes = bboxes(2,:);
    % frame = insertShape(JL,'Rectangle',bbox);
    % imshow(frame)
    % title('ROI for car detected with Cascade Object Detector');

    % extract a roi from z layer of point cloud
    x1 = bboxes(:,1);
    x2 = bboxes(:,1)+bboxes(:,3);
    y1 = bboxes(:,2);
    y2 = bboxes(:,2)+bboxes(:,4);
    ptCloudZRoi = ptCloud(y1:y2,x1:x2,3);
    % calculate the average z value or distance
    distance = mean(ptCloudZRoi(:),'omitnan');
    calc_distance(i) = distance;
    
    i=i+1;
    dis_string= sprintf('distance: %0.2f',distance);
    
    % overlay the average distance onto the image
    frame = insertObjectAnnotation(JL,'rectangle',bboxes,[dis_string,' meters']);
    if any(bboxes) ==1
        for ii=1:size(bboxes,1)
           % Add dimensions in pixels
            center_y=nr/2;
            center_x=nc/2;
            text_str = cell(4,1);
            text_str{1} = ['width: ',num2str(bboxes(ii,3)),'px'];
            text_str{2} = ['height: ',num2str(bboxes(ii,4)),'px'];
            text_str{3} = ['xLoc:',num2str(bboxes(ii,1)-center_x),'px'];
            text_str{4} = ['yLoc:',num2str(center_y-bboxes(ii,2)),'px'];
            pos = [bboxes(ii,1) bboxes(ii,4)+bboxes(ii,2)-5;bboxes(ii,1) bboxes(ii,4)+bboxes(ii,2)+10;bboxes(ii,1) bboxes(ii,4)+bboxes(ii,2)+25;bboxes(ii,1) bboxes(ii,4)+bboxes(ii,2)+40];
            frame = insertText(frame,pos,text_str,'FontSize',16,'BoxColor',[0 0 0],'BoxOpacity',0,'TextColor','cyan','Font','Arial Bold');
        end
    end
    
%   figure
%   imshow(frame)
%   title('Depth of car extracted from Point Cloud');
%show video
step(VP,frame);
%Write Video
step(VR,frame);
end
%show log of detected distances
figure;
plot(calc_distance)
title('Distance Between Cameras and Car')
xlabel('time (t)')
ylabel('distance (d)')

%Close Video Writer
release(VR);
release(LV)
release(RV)