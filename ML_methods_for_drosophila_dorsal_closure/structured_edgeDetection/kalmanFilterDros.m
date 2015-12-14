

function kalmanFilterDros()



%% variables and parameters
dt = 1;  %our sampling rate
S_frame = 1; End_frame=30; %starting and end frame for building velocity model
u = .005; % define acceleration magnitude
HexAccel_noise_mag = .1; %process noise: the variability in how fast the point is speeding up (stdv of acceleration: meters/sec^2)
tkn_x = 1;  %measurement noise in the horizontal direction (x axis).
tkn_y = 1;  %measurement noise in the horizontal direction (y axis).
Ez = [tkn_x 0; 0 tkn_y];
Ex = [dt^4/4 0 dt^3/2 0; ...
    0 dt^4/4 0 dt^3/2; ...
    dt^3/2 0 dt^2 0; ...
    0 dt^3/2 0 dt^2].*HexAccel_noise_mag^2; % Ex convert the process noise (stdv) into covariance matrix
P = Ex; % estimate of initial position variance (covariance matrix)

%% Define update equations in 2-D! (Coefficent matrices): A physics based model for where we expect the point to be [state transition (state + velocity)] + [input control (acceleration)]
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; %state update matrice
B = [(dt^2/2); (dt^2/2); dt; dt];
C = [1 0 0 0; 0 1 0 0];  %this is our measurement function C, that we apply to the state estimate Q to get our expect next/new measurement

%% initize result variables
% Initialize for speed
Q_loc = []; % ACTUAL point motion path
vel = []; % ACTUAL point velocity
Q_loc_meas = []; % the point path extracted by the tracking algo

%% initize estimation variables
Q_loc_estimate = []; %  position estimate
vel_estimate = []; % velocity estimate
P_estimate = P;
predic_state = [];
predic_var = [];
r = 5; % r is the radius of the plotting circle
j=0:.01:2*pi; %to make the plotting circle

%% images to be tracked
close all;
% set(0,'DefaultFigureWindowStyle','docked')
drosVideo = loadtiff('../images/DDC1_all.tif');

%% parameters
path = '/media/root/WORK/drosophila/DROS/data/';
numPts=1000;
colors = rand(3,numPts);
val = 10;
highVal = 10000;
indexSet = highVal*ones(numPts,End_frame);
CM_idx=[];
%% get correspondences for the first set of frames
for t = S_frame:End_frame
    img_tmp = double(drosVideo(:,:,t));
    img = img_tmp(:,:,1);
     h = fspecial('gaussian',3,5);
    imSmooth1 = imfilter(img_tmp,h);
%     imSmooth1 = imgaussfilt(img_tmp,5);
    corners = detectHarrisFeatures(imSmooth1);
    [desc, points] = extractFeatures(drosVideo(:,:,t), corners.Location);
    CM_idx1{t} = points;
    descriptors{t} = desc;
    if t==1
        indexSet(1:size(points,1),1) =1:size(points,1);
    else
        [indexPairs metric] = matchFeatures(descriptors{t-1},descriptors{t},'MatchThreshold',80,'MaxRatio',0.65);
         [aa,indUniquePM1,indUniquePM2] = unique(indexPairs(:,2),'rows'); %increase thi5
        countPM = accumarray(indUniquePM2,1);
        indDup = find(countPM>1);
        toDel=[];
        for a=1:length(indDup),  
            dupSet = find(indUniquePM2==indDup(a)); 
            if metric(dupSet(1))> metric(dupSet(1))
                toDel = [toDel; dupSet(1)];
            else
                toDel = [toDel; dupSet(2)];
            end
        end
        indexPairs(toDel,:) = [];
        
        tranformInd = indexSet(indexPairs(:,1),1);
        indexSet(tranformInd,t) = indexPairs(:,2);
        %% non-unique matches
        if 0
            figure; showMatchedFeatures(drosVideo(:,:,t-1), drosVideo(:,:,t), CM_idx{t-1}(indexPairs(:,1),:), CM_idx{t}(indexPairs(:, 2),:));
%             im1 = 1; im2 = t;
%             indSubset = indexSet(:,im1:im2);
%             [r,c] = find(indSubset>=highVal);
%             uniqueR = unique(r); indSubset(uniqueR,:)=[];
% 
%             figure; showMatchedFeatures(drosVideo(:,:,im1), drosVideo(:,:,im2), CM_idx{im1}(indSubset(:,1),:), CM_idx{im2}(indSubset(:, end),:));
        end
    end
end

% [row, col]= find(indexSet<highVal);
% uniqueRow = unique(row);
% indexSet(uniqueRow,:)=[];
% for t = S_frame:End_frame
%     CM_idx{t} = CM_idx{t}(indexSet(:,t),:);
% end

%% build a velocity model for the first nunmPts points
for t = S_frame:End_frame
    numPts=300;
    % load the image
    img_tmp = double(drosVideo(:,:,t));
    img = img_tmp(:,:,1);
    % load the given tracking
       h = fspecial('gaussian',3,5);
        imSmooth1 = imfilter(img_tmp,h);
    % corners = detectHarrisFeatures(imSmooth1);
    [~,indSort] = sort(corners.Metric,'descend');
    cornersSorted = corners.Location(indSort,:);
    cornersThresh = cornersSorted(1:numPts,:);
    CM_idx(t,:) = [mean(cornersThresh(:,2)) mean(cornersThresh(:,1))];
    if 0
        figure, imshow(drosVideo(:,:,i),[]);
        hold on, plot(CM_idx(t,2),CM_idx(t,1),'*g');
    end
    % hold on, plot(cornersThresh(:,1),cornersThresh(:,2),'*g');
    
    Q= [CM_idx(S_frame,1); CM_idx(S_frame,2); 0; 0]; %initized state--it has four components: [positionX; positionY; velocityX; velocityY] of the point
    Q_estimate = Q;  %estimate of initial location estimation of where the point is (what we are updating)
    
    Q_loc_meas(:,t) = [ CM_idx(t,1); CM_idx(t,2)];
    Q_loc_meas_all{t} = [cornersThresh(:,2) cornersThresh(:,1)]';
    Q_estimate_all = [Q_loc_meas_all{t}; zeros(2,numPts)];
    B_all = repmat(B,1,numPts);
    %% do the kalman filter
    
    % Predict next state of the point with the last state and predicted motion.
    Q_estimate = A * Q_estimate + B * u;
    Q_estimate_all = A * Q_estimate_all + B_all * u;
    predic_state = [predic_state; Q_estimate(1)] ;
    %predict next covariance
    P = A * P * A' + Ex;
    predic_var = [predic_var; P] ;
    % predicted Ninja measurement covariance
    % Kalman Gain
    K = P*C'*inv(C*P*C'+Ez);
    % Update the state estimate.
    if ~isnan(Q_loc_meas(:,t))
        Q_estimate = Q_estimate + K * (Q_loc_meas(:,t) - C * Q_estimate);
        Q_estimate_all = Q_estimate_all + K * (Q_loc_meas_all{t} - C * Q_estimate_all);
    end
    % update covariance estimation.
    P =  (eye(4)-K*C)*P;
    
    %% Store data
    Q_loc_estimate = [Q_loc_estimate; Q_estimate(1:2)];
    vel_estimate = [vel_estimate; Q_estimate(3:4)];
    %     CM_idx(t+1,:) = [Q_estimate(1:2)];
    %% plot the images with the  tracking
    if 1
        figure, imshow(img,[]);
        hold on;
        %         plot(r*sin(j)+Q_loc_meas(2,t),r*cos(j)+Q_loc_meas(1,t),'.g'); % the actual tracking
        %         plot(r*sin(j)+Q_estimate(2),r*cos(j)+Q_estimate(1),'.r'); % the kalman filtered tracking
        
        plot(Q_loc_meas_all{t}(2,:),Q_loc_meas_all{t}(1,:),'*g');
        plot(Q_estimate_all(2,:),Q_estimate_all(1,:),'*r');
        hold off
        pause(0.1)
    end
end


for t = End_frame:size(drosVideo,3)
    img_tmp = double(drosVideo(:,:,t));
    img = img_tmp(:,:,1);
    % load the given tracking
    Q_loc_meas(:,t) = [ CM_idx(t,1); CM_idx(t,2)];
    
    %% Redefine params based on displacement
    dt = 1; %t - End_frame+1;
    Ex = [dt^4/4 0 dt^3/2 0; ...
        0 dt^4/4 0 dt^3/2; ...
        dt^3/2 0 dt^2 0; ...
        0 dt^3/2 0 dt^2].*HexAccel_noise_mag^2; % Ex convert the process noise (stdv) into covariance matrix
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; %state update matrice
    B = [(dt^2/2); (dt^2/2); dt; dt];
    
    %% do the kalman filter
    
    % Predict next state of the point with the last state and predicted motion.
    Q_estimate = A * Q_estimate + B * u;
    predic_state = [predic_state; Q_estimate(1)] ;
    %predict next covariance
    P = A * P * A' + Ex;
    predic_var = [predic_var; P] ;
    % predicted Ninja measurement covariance
    % Kalman Gain
    K = P*C'*inv(C*P*C'+Ez);
    % Update the state estimate.
    if ~isnan(Q_loc_meas(:,t))
        Q_estimate = Q_estimate + K * (Q_loc_meas(:,t) - C * Q_estimate);
    end
    % update covariance estimation.
    P =  (eye(4)-K*C)*P;
    
    %% Store data
    Q_loc_estimate = [Q_loc_estimate; Q_estimate(1:2)];
    vel_estimate = [vel_estimate; Q_estimate(3:4)];
    CM_idx(t+1,:) = [Q_estimate(1:2)]';
    %% plot the images with the  tracking
    im(img);
    hold on;
    plot(r*sin(j)+Q_loc_meas(2,t),r*cos(j)+Q_loc_meas(1,t),'.g'); % the actual tracking
    plot(r*sin(j)+Q_estimate(2),r*cos(j)+Q_estimate(1),'.r'); % the kalman filtered tracking
    hold off
    pause(0.1)
    
end

if 0
        inverseMap = 1:max(indexPairs(:,2));
        inverseMap(indexPairs(:,2)) = 1:size(indexPairs(:,2),1);
        indexSet(inverseMap(indexPairs(:,1)),t) = indexPairs(:,2);
%         indexSetUnique = indexSet(:,1); indexSetUnique(toDel,:) = [];
         
    
    
end
