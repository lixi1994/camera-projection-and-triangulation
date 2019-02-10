profile on

%% load data
clear;
%load data
load vue2CalibInfo.mat
load vue4CalibInfo.mat
load Subject4-Session3-Take4_mocapJoints.mat
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';

%read camera parameters
Pmat2 = vue2.Pmat;
Rmat2 = vue2.Rmat;
Kmat2 = vue2.Kmat;
radial2 = vue2.radial;
C2 = vue2.position;
f2 = vue2.foclen;
F2 = [f2 0 0; 0 f2 0; 0 0 1];
Pmat4 = vue4.Pmat;
Rmat4 = vue4.Rmat;
Kmat4 = vue4.Kmat;
radial4 = vue4.radial;
C4 = vue4.position;
f4 = vue4.foclen;
F4 = [f4 0 0; 0 f4 0; 0 0 1];

% read video
N = size(mocapJoints,1); %frame #
vue2video = VideoReader(filenamevue2mp4);
vue4video = VideoReader(filenamevue4mp4);
% Subject_Camera2(N)=struct('cdata',[],'colormap',[]);
% Subject_Camera4(N) =struct('cdata',[],'colormap',[]);

%error
Error = zeros(N,12); %per frame per joint error
Error_uncorrected = zeros(N,12); %per frame per joint error
Error_2 = zeros(N,1); %error of combining 12 joints as one

%reconstructed 3D points
M = 0; % num of useful frame
for i = 1:N
    conf = mocapJoints(i,:,4);
    if (sum(conf)~=12)  %drop frame that doesn't have all conf = 1
        continue;
    end
    M = M+1;
end
Reconstructed_points = zeros(M,12,3);
m = 1; %index for Reconstructed_points

%% example frames - plot 2D and 3D points and draw skeleton
mocapFnum = 1000;
%read 3D world points
x = mocapJoints(mocapFnum,:,1);
y = mocapJoints(mocapFnum,:,2);
z = mocapJoints(mocapFnum,:,3);
WorldPoints = [x;y;z;ones(1,12)];
%uncorrected image points
ImagePoints2 = Kmat2*Pmat2*WorldPoints;
ImagePoints2 = bsxfun(@rdivide, ImagePoints2(1:2,:),ImagePoints2(3,:));
ImagePoints4 = Kmat4*Pmat4*WorldPoints;
ImagePoints4 = bsxfun(@rdivide, ImagePoints4(1:2,:),ImagePoints4(3,:));
%radial 2
FilmPoints2 = F2*Pmat2*WorldPoints;
FilmPoints2 = bsxfun(@rdivide, FilmPoints2(1:2,:),FilmPoints2(3,:));
FilmPointsCorrection2 = zeros(size(FilmPoints2));
%[FilmPointsCorrection2(1,:), FilmPointsCorrection2(2,:)] = performRadialCorrection(FilmPoints2(1,:),FilmPoints2(2,:),radial2);
[FilmPointsCorrection2(1,:), FilmPointsCorrection2(2,:)] = performRadialDistortion(FilmPoints2(1,:),FilmPoints2(2,:),radial2);
ImagePointsCorrection2 = Kmat2/F2*[FilmPointsCorrection2;ones(1,12)];
ImagePointsCorrection2 = ImagePointsCorrection2(1:2,:);
%radial 4
FilmPoints4 = F4*Pmat4*WorldPoints;
FilmPoints4 = bsxfun(@rdivide, FilmPoints4(1:2,:),FilmPoints4(3,:));
FilmPointsCorrection4 = zeros(size(FilmPoints4));
%[FilmPointsCorrection4(1,:), FilmPointsCorrection4(2,:)] = performRadialCorrection(FilmPoints4(1,:),FilmPoints4(2,:),radial4);
[FilmPointsCorrection4(1,:), FilmPointsCorrection4(2,:)] = performRadialDistortion(FilmPoints4(1,:),FilmPoints4(2,:),radial4);
ImagePointsCorrection4 = Kmat4/F4*[FilmPointsCorrection4;ones(1,12)];
ImagePointsCorrection4 = ImagePointsCorrection4(1:2,:);
%get video frame from vue2
vue2video.CurrentTime = (mocapFnum-1)*(50/100)/vue2video.FrameRate;
vid2Frame = readFrame(vue2video);
%get video frame from vue4
vue4video.CurrentTime = (mocapFnum-1)*(50/100)/vue4video.FrameRate;
vid4Frame = readFrame(vue4video);

% plot 2D skeleton
% vue2
figure(1);
image(vid2Frame);
for i= 1:3:size(ImagePointsCorrection2,2)
    hold on;
    plot(ImagePointsCorrection2(1,i:i+2),ImagePointsCorrection2(2,i:i+2),'c-*');
end
plot([ImagePointsCorrection2(1,1),ImagePointsCorrection2(1,4)],[ImagePointsCorrection2(2,1),ImagePointsCorrection2(2,4)],'c-*',[ImagePointsCorrection2(1,7),ImagePointsCorrection2(1,10)],[ImagePointsCorrection2(2,7),ImagePointsCorrection2(2,10)],'c-*');
p1 = (ImagePointsCorrection2(:,1)+ImagePointsCorrection2(:,4))/2;  % mid-shoulder
p2 = (ImagePointsCorrection2(:,7)+ImagePointsCorrection2(:,10))/2; % mid-hip
points = [p1 p2];
plot(points(1,:),points(2,:),'c-*');
% vue4
figure(2);
image(vid4Frame);
for i= 1:3:size(ImagePointsCorrection4,2)
    hold on;
    plot(ImagePointsCorrection4(1,i:i+2),ImagePointsCorrection4(2,i:i+2),'y-*');
end
plot([ImagePointsCorrection4(1,1),ImagePointsCorrection4(1,4)],[ImagePointsCorrection4(2,1),ImagePointsCorrection4(2,4)],'y-*',[ImagePointsCorrection4(1,7),ImagePointsCorrection4(1,10)],[ImagePointsCorrection4(2,7),ImagePointsCorrection4(2,10)],'y-*');
p1 = (ImagePointsCorrection4(:,1)+ImagePointsCorrection4(:,4))/2;  % mid-shoulder
p2 = (ImagePointsCorrection4(:,7)+ImagePointsCorrection4(:,10))/2; % mid-hip
points = [p1 p2];
plot(points(1,:),points(2,:),'y-*');

% plot 3D skeleton
% only forward distortion
[predicted, distances] = triangulateDLT(Kmat2,Rmat2,C2',ImagePointsCorrection2,Kmat4,Rmat4,C4',ImagePointsCorrection4);
% error
diff = predicted(:,:)-WorldPoints(1:3,:);
error_distortion = sqrt(sum(diff.^2,1));
%plot
figure(3);
for i= 1:3:size(predicted,2)
    hold on;
    plot3(predicted(1,i:i+2),predicted(2,i:i+2),predicted(3,i:i+2),'b-*');
end
points = [predicted(:,1) predicted(:,4) predicted(:,7) predicted(:,10)]; % shoulders & hips
plot3(points(1,1:2),points(2,1:2),points(3,1:2),'b-*',points(1,3:4),points(2,3:4),points(3,3:4),'b-*');
p1 = (predicted(:,1)+predicted(:,4))/2; % mid-shoulder
p2 = (predicted(:,7)+predicted(:,10))/2; % mid-hip
points = [p1 p2];
plot3(points(1,:),points(2,:),points(3,:),'b-*');

% % uncorrected points
% [predicted, distances] = triangulateDLT(Kmat2,Rmat2,C2',ImagePoints2,Kmat4,Rmat4,C4',ImagePoints4);
% % error
% diff = predicted(:,:)-WorldPoints(1:3,:);
% error_uncorrected = sqrt(sum(diff.^2,1));
% % plot
% for i= 1:3:size(predicted,2)
%     plot3(predicted(1,i:i+2),predicted(2,i:i+2),predicted(3,i:i+2),'r-*');
% end
% points = [predicted(:,1) predicted(:,4) predicted(:,7) predicted(:,10)]; % shoulders & hips
% plot3(points(1,1:2),points(2,1:2),points(3,1:2),'r-*',points(1,3:4),points(2,3:4),points(3,3:4),'r-*');
% p1 = (predicted(:,1)+predicted(:,4))/2; % mid-shoulder
% p2 = (predicted(:,7)+predicted(:,10))/2; % mid-hip
% points = [p1 p2];
% plot3(points(1,:),points(2,:),points(3,:),'r-*');

% forward distortion and backward correction
IF2 = zeros(size(FilmPoints2));
IF4 = zeros(size(FilmPoints4));
FilmPointsCorrection2 = F2/Kmat2*[ImagePointsCorrection2;ones(1,12)];
FilmPointsCorrection4 = F4/Kmat4*[ImagePointsCorrection4;ones(1,12)];
FilmPointsCorrection2 = FilmPointsCorrection2(1:2,:);
FilmPointsCorrection4 = FilmPointsCorrection4(1:2,:);
[IF2(1,:), IF2(2,:)] = performRadialCorrection(FilmPointsCorrection2(1,:),FilmPointsCorrection2(2,:),radial2);
[IF4(1,:), IF4(2,:)] = performRadialCorrection(FilmPointsCorrection4(1,:),FilmPointsCorrection4(2,:),radial4);
IP2 = Kmat2/F2*[IF2;ones(1,12)];
IP4 = Kmat4/F4*[IF4;ones(1,12)];
IP2 = IP2(1:2,:);
IP4 = IP4(1:2,:);
[predicted, distances] = triangulateDLT(Kmat2,Rmat2,C2',IP2,Kmat4,Rmat4,C4',IP4);
% error
diff = predicted(:,:)-WorldPoints(1:3,:);
error_uncorrected = sqrt(sum(diff.^2,1));
% plot
for i= 1:3:size(predicted,2)
    plot3(predicted(1,i:i+2),predicted(2,i:i+2),predicted(3,i:i+2),'r-*');
end
points = [predicted(:,1) predicted(:,4) predicted(:,7) predicted(:,10)]; % shoulders & hips
plot3(points(1,1:2),points(2,1:2),points(3,1:2),'r-*',points(1,3:4),points(2,3:4),points(3,3:4),'r-*');
p1 = (predicted(:,1)+predicted(:,4))/2; % mid-shoulder
p2 = (predicted(:,7)+predicted(:,10))/2; % mid-hip
points = [p1 p2];
plot3(points(1,:),points(2,:),points(3,:),'r-*');

% world 3D
predicted = WorldPoints(1:3,:);
for i= 1:3:size(predicted,2)
    plot3(predicted(1,i:i+2),predicted(2,i:i+2),predicted(3,i:i+2),'g-o');
end
points = [predicted(:,1) predicted(:,4) predicted(:,7) predicted(:,10)]; % shoulders & hips
plot3(points(1,1:2),points(2,1:2),points(3,1:2),'g-o',points(1,3:4),points(2,3:4),points(3,3:4),'g-o');
p1 = (predicted(:,1)+predicted(:,4))/2; % mid-shoulder
p2 = (predicted(:,7)+predicted(:,10))/2; % mid-hip
points = [p1 p2];
plot3(points(1,:),points(2,:),points(3,:),'g-o');
rotate3d on

%% compute error for each frame
h = waitbar(0,'please wait...');
for mocapFnum = 1:N %set frame #
    waitbar(mocapFnum/N);
    %% 3D world points to 2D image points
    %read 3D joint data
    x = mocapJoints(mocapFnum,:,1);
    y = mocapJoints(mocapFnum,:,2);
    z = mocapJoints(mocapFnum,:,3);
    conf = mocapJoints(mocapFnum,:,4);
    if (sum(conf)~=12)  %drop frame that doesn't have all conf = 1
        continue;
    end
    WorldPoints = [x;y;z;ones(1,12)];
    %project to vue2
    ImagePoints2 = Kmat2*Pmat2*WorldPoints;
    ImagePoints2 = bsxfun(@rdivide, ImagePoints2(1:2,:),ImagePoints2(3,:));
    %project to vue4
    ImagePoints4 = Kmat4*Pmat4*WorldPoints;
    ImagePoints4 = bsxfun(@rdivide, ImagePoints4(1:2,:),ImagePoints4(3,:));
    %radial correction
    %radial 2
    FilmPoints2 = F2*Pmat2*WorldPoints;
    FilmPoints2 = bsxfun(@rdivide, FilmPoints2(1:2,:),FilmPoints2(3,:));
    FilmPointsCorrection2 = zeros(size(FilmPoints2));
    [FilmPointsCorrection2(1,:), FilmPointsCorrection2(2,:)] = performRadialCorrection(FilmPoints2(1,:),FilmPoints2(2,:),radial2);
    ImagePointsCorrection2 = Kmat2/F2*[FilmPointsCorrection2;ones(1,12)];
    ImagePointsCorrection2 = ImagePointsCorrection2(1:2,:);
    %radial 4
    FilmPoints4 = F4*Pmat4*WorldPoints;
    FilmPoints4 = bsxfun(@rdivide, FilmPoints4(1:2,:),FilmPoints4(3,:));
    FilmPointsCorrection4 = zeros(size(FilmPoints4));
    [FilmPointsCorrection4(1,:), FilmPointsCorrection4(2,:)] = performRadialCorrection(FilmPoints4(1,:),FilmPoints4(2,:),radial4);
    ImagePointsCorrection4 = Kmat4/F4*[FilmPointsCorrection4;ones(1,12)];
    ImagePointsCorrection4 = ImagePointsCorrection4(1:2,:);

    %% plot 2D points on corresponding frame
%     try %get frames
%         %get video frame from vue2
%         vue2video.CurrentTime = (mocapFnum-1)*(50/100)/vue2video.FrameRate;
%         vid2Frame = readFrame(vue2video);
%         %get video frame from vue4
%         vue4video.CurrentTime = (mocapFnum-1)*(50/100)/vue4video.FrameRate;
%         vid4Frame = readFrame(vue4video);
%     catch %some frame may be lost
%         continue
%     end
%     % plot points on frame 
%     pos = ImagePoints2';
%     V2F = insertMarker(vid2Frame,pos,'*','color','blue','size',5);
%     pos = ImagePoints4';
%     V4F = insertMarker(vid4Frame,pos,'*','color','blue','size',5);
%     pos = ImagePointsCorrection2';
%     V2C = insertMarker(V2F,pos,'*','color','red','size',5);
%     imshow(V2C);
%     Subject_Camera2(mocapFnum) = getframe;
%     pos = ImagePointsCorrection4';
%     V4C = insertMarker(V4F,pos,'*','color','red','size',5);
%     imshow(V4C);
%     Subject_Camera4(mocapFnum) = getframe;
    
    %% triangulation back to 3D points
    %uncorrected image points
    [predicted, distances] = triangulateDLT(Kmat2,Rmat2,C2',ImagePoints2,Kmat4,Rmat4,C4',ImagePoints4);
    %error
    diff = predicted(:,:)-WorldPoints(1:3,:);
    error = sqrt(sum(diff.^2,1));
    Error_uncorrected(mocapFnum,:) = error;
    % case 2: nonlinear distortion in forward projection 
    % but no correction in backward projection
    [predicted, distances] = triangulateDLT(Kmat2,Rmat2,C2',ImagePointsCorrection2,Kmat4,Rmat4,C4',ImagePointsCorrection4);
    Reconstructed_points(m,:,:) = predicted';
    m = m+1;
    %error
    diff = predicted(:,:)-WorldPoints(1:3,:);
    error = sqrt(sum(diff.^2,1));
    Error(mocapFnum,:) = error;
    predicted_in_one = sum(predicted,2);
    world_in_one = sum(WorldPoints(1:3,:),2);
    diff_in_one = predicted_in_one-world_in_one;
    error_in_one = sqrt(sum(diff_in_one.^2));
    Error_2(mocapFnum,:) = error_in_one;

end
close(h);

%play the movie
%movie(Subject_Camera2,10);
%movie(Subject_Camera4,10);
%store the movie
% v2 = VideoWriter('vue2.avi');
% open(v2);
% writeVideo(v2,Subject_Camera2);
% close(v2);
% v4 = VideoWriter('vue4.avi');
% open(v4);
% writeVideo(v4,Subject_Camera4);
% close(v4);

%save reconstructed points
save('Reconstructed_points.mat','Reconstructed_points');

%error evaluation
Error = Error(any(Error,2),:);  %remove rows with all 0s
Error_uncorrected = Error_uncorrected(any(Error_uncorrected,2),:);
Error_2 = Error_2(any(Error_2,2),:);
%evaluation of uncorrected image points
num = size(Error_uncorrected,1); % valid frame number
Mean_Err_uncorrected = sum(Error_uncorrected,1)/num;
Min_Err_uncorrected = min(Error_uncorrected,[],1);
Max_Err_uncorrected = max(Error_uncorrected,[],1);
Median_Err_uncorrected = median(Error_uncorrected,1);
Std_Error_uncorrected = std(Error_uncorrected,0,1);
Sum_Error_uncorrected = sum(Error_uncorrected,2);
figure(4);
plot(1:num,Sum_Error_uncorrected(:,1),'o-');
xlabel('frame');
ylabel('error');
%evaluation of case 2
num = size(Error,1); % valid frame number
Mean_Err = sum(Error,1)/num;
Min_Err = min(Error,[],1);
Max_Err = max(Error,[],1);
Median_Err = median(Error,1);
Std_Error = std(Error,0,1);
Sum_Error = sum(Error,2);
figure(5);
plot(1:num,Sum_Error(:,1),'o-');
xlabel('frame');
ylabel('error');
%evaluation, all joint points as one
num = size(Error_2,1); % valid frame number
Mean_Err_2 = sum(Error_2,1)/num;
Min_Err_2 = min(Error_2,[],1);
Max_Err_2 = max(Error_2,[],1);
Median_Err_2 = median(Error_2,1);
Std_Error_2 = std(Error_2,0,1);

p = profile('info');
save myprofiledata p;
