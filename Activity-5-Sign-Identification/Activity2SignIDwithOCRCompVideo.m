% Create object to read video frames
%vidReader = vision.VideoFileReader('rawActivity2VideoTeamSelected4.mp4');
vidReader = vision.VideoFileReader('rawActivity3Video.mp4');

%Change data type
vidReader.VideoOutputDataType = 'double';

%Create object to Write Video
%videoFReader = vision.VideoFileReader('rawActivity2VideoTeamSelected4.mp4');
videoFReader = vision.VideoFileReader('rawActivity3Video.mp4');
%videoFWriter = vision.VideoFileWriter('activity2Sign_CSULAYel2_7.mp4','FrameRate',videoFReader.info.VideoFrameRate,'FileFormat','MPEG4');
videoFWriter = vision.VideoFileWriter('activity2Sign_CSULAYel14.mp4','FrameRate',videoFReader.info.VideoFrameRate,'FileFormat','MPEG4');

%% % Create VideoPlayer
vidPlayer = vision.DeployableVideoPlayer;
videoPlayer = vision.VideoPlayer;

%% % Frame Counter
i=1;
%% Create blob analysis object 
%Blob analysis object further filters the detected foreground by rejecting blobs which contain fewer
% than 900 pixels.
blobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
    'AreaOutputPort', false, 'CentroidOutputPort', false, ...
    'MinimumBlobArea', 100);
%%
while ~isDone(vidReader)

    %Get the next frame
    videoFrame = step(vidReader);

    % Find location of traffic sign
    % Convert image from rgb to hsv
    frameHsv = rgb2hsv(videoFrame);
    I= createMask(frameHsv);

    if i == 235 
        %imwrite(frameHsv,'stophsv.jpg');
        %imwrite(I,'BWStop.jpg');
        %imwrite(bm,'MORPHstop.jpg');
        imwrite(result,'FinalstopBound.jpg');
        %imwrite(videoFrame,'Finalstop.jpg');

    end

    % Set up blob analysis object
    % Threshold image
    % Perform morphological opening to get rid of background noise
    bm = imopen(I,strel('disk',1));
    bm = imclose(bm,strel('octagon',15));

    bbox = step(blobAnalysis, bm);

    result = videoFrame;

    if any(bbox)==1
    % Iterate through every bounding box per frame
        for i=1:size(bbox,1)
            
            % Perform bwmorph within bbox
            cropped_videoFrame = imcrop(videoFrame,bbox(i,:));
            cropped_bw = im2bw(cropped_videoFrame);
            bw = bwmorph(cropped_bw,'thin',Inf);

            % Perform OCR to find any text matching the letters in STOP
            ocrResults = ocr(bw,'TextLayout','Block','CharacterSet','STOP');

            % Check if OCR detected any words
            if ~isempty(ocrResults.Words)

                % Sort the word confidences
                [sortedConf, sortedIndex] = sort(ocrResults.WordConfidences, 'descend');

                % Keep indices associated with non-NaN confidences values
                indexesNaNsRemoved = sortedIndex( ~isnan(sortedConf) );

                % Get the top index
                topIndex = indexesNaNsRemoved(1:1);

                % Search for matches for the string 'stop'
                locatedBoxes = locateText(ocrResults, '.*stop.*', 'IgnoreCase', true, 'UseRegexp', true);

                % Annotate image with character confidences
                str      = sprintf('confidence = %.2f%%', ocrResults.WordConfidences(topIndex)*100);

                % Box is green if 'stop' was found
                if size(locatedBoxes,1) > 0
                    result = insertObjectAnnotation(result, 'rectangle', bbox(i,:), str, 'Color', 'red','LineWidth',5,'FontSize',22);
                
                else
                    % Box is yellow if other words were found
                    result = insertObjectAnnotation(result, 'rectangle', bbox(i,:), 'Low Confidence', 'Color', 'yellow','LineWidth',5,'FontSize',22);
                end          
            else
                % Box is white if no words were found
                result = insertObjectAnnotation(result, 'rectangle', bbox(i,:), 'No Confidence', 'Color', 'white','LineWidth',5,'FontSize',22);
            end
            
            % Add dimensions in pixels
            [frame_height,frame_width,depth]=size(videoFrame);
            center_y=frame_height/2;
            center_x=frame_width/2;
            text_str = cell(4,1);
            text_str{1} = ['width: ',num2str(bbox(i,3)),'px'];
            text_str{2} = ['height: ',num2str(bbox(i,4)),'px'];
            text_str{3} = ['xLoc:',num2str(bbox(i,1)-center_x),'px'];
            text_str{4} = ['yLoc:',num2str(center_y-bbox(i,2)),'px'];
            pos = [bbox(i,1) bbox(i,4)+bbox(i,2)-5;bbox(i,1) bbox(i,4)+bbox(i,2)+10;bbox(i,1) bbox(i,4)+bbox(i,2)+25;bbox(i,1) bbox(i,4)+bbox(i,2)+40];
            result = insertText(result,pos,text_str,'FontSize',16,'BoxColor',[0 0 0],'BoxOpacity',0,'TextColor','cyan','Font','Arial Bold');
            
        end

    %Play in the video player
    step(vidPlayer,result);
    step(videoFWriter, result);

    else 
        step(vidPlayer, videoFrame);
       %     step(fgPlayer,videoFrame);        
        step(videoFWriter, videoFrame);
    end

        i=i+1;
    
end
    