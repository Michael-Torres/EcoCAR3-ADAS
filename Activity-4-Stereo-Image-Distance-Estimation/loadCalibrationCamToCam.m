function calib = loadCalibrationCamToCam(~)

% open file
fid = fopen('calib_cam_to_cam.txt','r');

if fid<0
  calib = [];
  return;
end

% read corner distance
calib.cornerdist = readVariable(fid,'corner_dist',1,1);

% read all cameras (maximum: 100)
for cam=1:100
  
  % read variables
  S_      = readVariable(fid,['S_' num2str(cam-1,'%02d')],1,2); %original size image
  K_      = readVariable(fid,['K_' num2str(cam-1,'%02d')],3,3); %calibration matrices
  D_      = readVariable(fid,['D_' num2str(cam-1,'%02d')],1,5); %distortion coeff
  R_      = readVariable(fid,['R_' num2str(cam-1,'%02d')],3,3); %rotation from camera 0 to cam
  T_      = readVariable(fid,['T_' num2str(cam-1,'%02d')],3,1); %translation from cam 0 to cam
  S_rect_ = readVariable(fid,['S_rect_' num2str(cam-1,'%02d')],1,2); %image size after rect
  R_rect_ = readVariable(fid,['R_rect_' num2str(cam-1,'%02d')],3,3); %rectifying rotation matrix
  P_rect_ = readVariable(fid,['P_rect_' num2str(cam-1,'%02d')],3,4); %projection matrix after rect
  
  % calibration for this cam completely found?
  if isempty(S_) || isempty(K_) || isempty(D_) || isempty(R_) || isempty(T_)
    break;
  end
  
  % write calibration
  calib.S{cam} = S_;
  calib.K{cam} = K_;
  calib.D{cam} = D_;
  calib.R{cam} = R_;
  calib.T{cam} = T_;
  
  % if rectification available
  if ~isempty(S_rect_) && ~isempty(R_rect_) && ~isempty(P_rect_)
    calib.S_rect{cam} = S_rect_;
    calib.R_rect{cam} = R_rect_;
    calib.P_rect{cam} = P_rect_;
  end
end

% close file
fclose(fid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function A = readVariable(fid,name,M,N)

% rewind
fseek(fid,0,'bof');

% search for variable identifier
success = 1;
while success>0
  [str,success] = fscanf(fid,'%s',1);
  if strcmp(str,[name ':'])
    break;
  end
end

% return if variable identifier not found
if ~success
  A = [];
  return;
end

% fill matrix
A = zeros(M,N);
for m=1:M
  for n=1:N
    [val,success] = fscanf(fid,'%f',1);
    if success
      A(m,n) = val;
    else
      A = [];
      return;
    end
  end
end


