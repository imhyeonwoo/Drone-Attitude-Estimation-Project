% -------------------------------------------------------------------------
% 2025 신호및시스템 ARS PACKET
% ROS통신으로 저장한 .bag 파일에서 우리가 이용하는 topic을 고른 후
% .mat file로 데이터 변수 저장하는 코드

% 학번: 202112305
% 이름: 임현우
% -------------------------------------------------------------------------
clc; clear; close all;
tic

%% 1) Open BAG file
% .bag 파일 저장
bag = rosbag('signal_exp_data.bag');   % signal_exp_data.bag 경로
               
% Display available topics
disp('Available topics:');
disp(bag.AvailableTopics.Properties.RowNames);


%% 2) Select the topics of interest
imuBag   = select(bag, 'Topic', '/xsens/imu_mode20'); % IMU sensor 값 (Accel + Gyro)
stateBag = select(bag, 'Topic', '/dsr01a0509/state'); % robot-arm의 state
poseBag  = select(bag, 'Topic', '/optitrack/drone/poseStamped'); 
% Reference로 사용할 Optitrack의 위치(NED) + 자세(Quaternion) 데이터


%% 3) Read messages (struct format)
imuMsgs   = readMessages(imuBag,   'DataFormat','struct');
stateMsgs = readMessages(stateBag, 'DataFormat','struct');
poseMsgs  = readMessages(poseBag,  'DataFormat','struct');


%% 4) Extract timestamps
% 모든 데이터가 동시에 rosbag record 시작 및 종료되었으므로, time-sync 거의 일치함

% Header-less topics: use MessageList.Time (xsens, robotarm)

imuTime   = imuBag.MessageList.Time;
stateTime = stateBag.MessageList.Time;

% PoseStamped: use Header.Stamp (integer + nanosecond -> float second)
% (optitrack)

numPose = numel(poseMsgs);
poseTime = zeros(numPose,1);
for i = 1:numPose
    poseTime(i) = double(poseMsgs{i}.Header.Stamp.Sec) + double(poseMsgs{i}.Header.Stamp.Nsec)*1e-9;
end


%% 5) Extract payload data
% 5.1-1) IMU: Float32MultiArray → cell of numeric vectors (xsens)
numImu = numel(imuMsgs);
imuData = cell(numImu,1);
for i = 1:numImu

    imuData{i} = imuMsgs{i}.Data;
    
end

% 5.2) PoseStamped: position only → Nx3 matrix (optitrack)
poseXYZ = zeros(numPose,3);
quatXYZW = zeros(numPose,4);

for i = 1:numPose

    p = poseMsgs{i}.Pose.Position;
    a = poseMsgs{i}.Pose.Orientation;
    poseXYZ(i,:) = [p.X, p.Y, p.Z];
    quatXYZW(i,:) = [a.X, a.Y, a.Z, a.W];

end


%% 6) Save results to .mat
save('packet_data.mat'); % .mat 파일로 저장



%% 7) Plot data
%% ----------------------------------------
% 위에서 저장한 데이터를 바탕으로 데이터 유효성 검사

% IMU cell → 행렬
nImu  = numel(imuData);
imuMat = zeros(nImu, numel(imuData{1}));
for k = 1:nImu, imuMat(k,:) = imuData{k}(:)'; end

accX = imuMat(:,1);  accY = imuMat(:,2);  accZ = imuMat(:,3);
gyroX = imuMat(:,4); gyroY = imuMat(:,5); gyroZ = imuMat(:,6);

tImu  = imuTime  - imuTime(1);
tPose = poseTime - poseTime(1);

% (1) 가속도·자이로 크기
figure('Name','IMU check','NumberTitle','off'); tiledlayout(2,1);
nexttile
plot(tImu, sqrt(accX.^2+accY.^2+accZ.^2)); grid on
title('|\ita|  (m/s^2)'); xlabel('time (s)')
nexttile
plot(tImu, sqrt(gyroX.^2+gyroY.^2+gyroZ.^2)); grid on
title('|\omega|  (deg/s)'); xlabel('time (s)')

% (2) Optitrack 쿼터니언 norm
figure('Name','Optitrack q-norm','NumberTitle','off');
plot(tPose, vecnorm(quatXYZW,2,2)); grid on
title('‖q‖'); xlabel('time (s)')

% (3) (IMU 6-axis plot: ax, ay, az, gx, gy, gz)
figure('Name','IMU 6-axis components','NumberTitle','off');
tiledlayout(6,1,"TileSpacing","compact","Padding","compact")

% 1) acc_x
nexttile
plot(tImu, accX); grid on
ylabel('acc_x (m/s^2)')

% 2) acc_y
nexttile
plot(tImu, accY); grid on
ylabel('acc_y (m/s^2)')

% 3) acc_z
nexttile
plot(tImu, accZ); grid on
ylabel('acc_z (m/s^2)')

% 4) gyro_x
nexttile
plot(tImu, gyroX); grid on
ylabel('gyro_x (deg/s)')

% 5) gyro_y
nexttile
plot(tImu, gyroY); grid on
ylabel('gyro_y (deg/s)')

% 6) gyro_z
nexttile
plot(tImu, gyroZ); grid on
ylabel('gyro_z (deg/s)')
xlabel('Time (s)')

sgtitle('IMU Data over Time')  % 공통 제목

% (4) Optitrack quaternion & Euler plot
% ─ 변환 준비 : quatXYZW = [x y z w]  → quatWXYZ = [w x y z]
quatWXYZ = [quatXYZW(:,4), quatXYZW(:,1:3)];    % MATLAB 형식 맞추기
eulRad   = quat2eul(quatWXYZ,'ZYX');            % [yaw pitch roll] in rad
eulDeg   = rad2deg(eulRad);                     % deg 로 변환

% ─ 그래프 : 4개 quaternion 성분 + 3개 Euler 각
figure('Name','Optitrack data','NumberTitle','off');
tiledlayout(2,1,"TileSpacing","compact","Padding","compact")

% (a) Quaternion components
nexttile
plot(tPose, quatXYZW(:,1), 'r', ...
     tPose, quatXYZW(:,2), 'g', ...
     tPose, quatXYZW(:,3), 'b', ...
     tPose, quatXYZW(:,4), 'k');
grid on; ylabel('q'); title('Quaternion components');
legend('q_x','q_y','q_z','q_w','Location','best');

% (b) Euler angles (roll-pitch-yaw)
nexttile
plot(tPose, eulDeg(:,3), 'b', ...
     tPose, eulDeg(:,2), 'g', ...
     tPose, eulDeg(:,1), 'r');
grid on; ylabel('deg'); xlabel('Time (s)');
title('Euler angles'); legend('Roll (\phi)','Pitch (\theta)','Yaw (\psi)','Location','best');


%% ----------------------------------------


toc