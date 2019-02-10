load Reconstructed_points.mat
load Subject4-Session3-Take4_mocapJoints.mat

N = size(mocapJoints,1); %frame #
M = size(Reconstructed_points,1); % useful frame #
m = 1; %index for Reconstruced points
figure
axis tight manual
ax = gca;
ax.NextPlot = 'replaceChildren';

%% plot3 movie
% to save time and memory, refresh SkeletonMovie every 1000 frames
SkeletonMovie(10)=struct('cdata',[],'colormap',[]);
j = 1; %index for movie
h = waitbar(0,'please wait...');
for mocapFnum = 6001:7000 %set frame #
    waitbar((mocapFnum-6000)/7000);
    %read 3D joint data
    x = mocapJoints(mocapFnum,:,1);
    y = mocapJoints(mocapFnum,:,2);
    z = mocapJoints(mocapFnum,:,3);
    conf = mocapJoints(mocapFnum,:,4);
    if (sum(conf)~=12)  %drop frame that doesn't have all conf = 1
        continue;
    end
    WorldPoints = [x;y;z];
    ReconstructedPoints = Reconstructed_points(m,:,:);
    m = m+1;
    
    %plot 3D
    %figure(3);
    clf;
    %reconstructed 3D points
    predicted = reshape(ReconstructedPoints,12,3,[])';
    for i= 1:3:size(predicted,2)
        plot3(predicted(1,i:i+2),predicted(2,i:i+2),predicted(3,i:i+2),'b-*');
        hold on;
    end
    points = [predicted(:,1) predicted(:,4) predicted(:,7) predicted(:,10)]; % shoulders & hips
    plot3(points(1,1:2),points(2,1:2),points(3,1:2),'b-*',points(1,3:4),points(2,3:4),points(3,3:4),'b-*');
    p1 = (predicted(:,1)+predicted(:,4))/2; % mid-shoulder
    p2 = (predicted(:,7)+predicted(:,10))/2; % mid-hip
    points = [p1 p2];
    plot3(points(1,:),points(2,:),points(3,:),'b-*');
    % world 3D
    predicted = WorldPoints;
    for i= 1:3:size(predicted,2)
        plot3(predicted(1,i:i+2),predicted(2,i:i+2),predicted(3,i:i+2),'g-o');
    end
    points = [predicted(:,1) predicted(:,4) predicted(:,7) predicted(:,10)]; % shoulders & hips
    plot3(points(1,1:2),points(2,1:2),points(3,1:2),'g-o',points(1,3:4),points(2,3:4),points(3,3:4),'g-o');
    p1 = (predicted(:,1)+predicted(:,4))/2; % mid-shoulder
    p2 = (predicted(:,7)+predicted(:,10))/2; % mid-hip
    points = [p1 p2];
    plot3(points(1,:),points(2,:),points(3,:),'g-o');
    
    SkeletonMovie(j) = getframe;
    j = j+1;
    
end
close(h);
%store the movie
v2 = VideoWriter('3Dpoints-6001-7000.avi');
open(v2);
writeVideo(v2,SkeletonMovie);
close(v2);