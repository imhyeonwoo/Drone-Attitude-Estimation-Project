%% -------------------------------------------------------------------------
% 2025 신호및시스템 ARS SCRIPT

% 학번: 202112305
% 이름: 임현우
%% ------------------------------------------------------------------------

clc; clear; close all;
tic
load('packet_data.mat')


%% --- Load Optitrack data ---
% 기준센서인 옵티트랙 데이터를 직관적인 오일러 자세각으로 변환. 즉, Quaternion -> Euler angle 변환
% MATLAB quat2eul expects [w x y z]
quatWXYZ = [quatXYZW(:,4), quatXYZW(:,1:3)]; 

% 'ZYX' sequence -> [yaw pitch roll] (rad)
eulRad = quat2eul(quatWXYZ, 'ZYX');     

% degree로 변환
eulDeg = rad2deg(eulRad); % [yaw pitch roll] 순서로 저장됨


%% --- Load and organize IMU data ---
% 기존 cell로 저장된 IMU 데이터 분리
N = numel(imuData);
imuMat = zeros(N,6);
for i = 1:N
    imuMat(i,:) = imuData{i}';
end
accX = imuMat(:,1);  % x축 가속도계 데이터 저장
accY = imuMat(:,2);  % y축 가속도계 데이터 저장
accZ = imuMat(:,3);  % z축 가속도계 데이터 저장
gyroX = imuMat(:,4); % x축 자이로 데이터 저장
gyroY = imuMat(:,5); % y축 자이로 데이터 저장
gyroZ = imuMat(:,6); % z축 자이로 데이터 저장


%% --- IMU time ---
T = imuTime - imuTime(1);
dt = mean(diff( T ));  %  IMU(Xsens) 센서는 대략 60Hz로 데이터를 받음 ( ~ 1/60 = 0.0167)


%% --- Optitrack(reference) time ---
T_2 = poseTime - poseTime(1);

% 두 시스템간의 시간차로 미세한 시간 비동기화가 일어나나, T와 T_2는 거의 일치하는 것으로 가정하고 진행할 것.

%% --- Bias removal on gyro using initial stationary periods ---
% PR_#1 % 초기 30초 정지상태 평균값을 이용하여 자이로 bias 제거 알고리즘 작성하시오!
% ---- 0) 설정 ----------------------------------------------------------
biasIdx  = T < 30;                       % 정지 구간
gyroBias = mean(imuMat(biasIdx,4:6),1);  % [Bx By Bz]  deg/s
gyroX_c = gyroX - gyroBias(1);
gyroY_c = gyroY - gyroBias(2);
gyroZ_c = gyroZ - gyroBias(3);

% ---- 1) 축별 원본 vs. 보정 ------------------------------------------
axesLbl = {'x','y','z'};
rawGyro = {gyroX, gyroY, gyroZ};
corGyro = {gyroX_c, gyroY_c, gyroZ_c};

figure('Name', 'Gyro All Axes', 'NumberTitle', 'off');  % 하나의 figure 창 생성

for k = 1:3
    subplot(3,1,k);  % 3행 1열 subplot 중 k번째
    plot(T, rawGyro{k}, 'Color', [0.8 0 0]); hold on;          % 원본(빨강)
    plot(T, corGyro{k}, 'k');                                 % 보정(검정)
    yline(gyroBias(k), '--r', 'Bias');
    grid on; xlabel('Time (s)'); ylabel('deg/s');
    legend('raw','bias-removed');
    title(sprintf('\\omega_{%s}  (bias = %.3f deg/s)', ...
          axesLbl{k}, gyroBias(k)));
end



%% FFT(Fast Fourier Transform)을 통해 주파수 해석 수행후, 저주파 통과필터 대역 설정

% Optitrack data (reference)의 로봇팔 미세 진동으로 인한 LPF 적용
Fs2 = 100;                    % Sampling frequency
period2 = 1/Fs2;              % Sampling period 
L2 = numel(poseMsgs);         % Length of signal
t2 = (0:L2 - 1)*period2;      % Time vector
opti_fftr = fft(eulDeg(:,3)); opti_fftp = fft(eulDeg(:,2));  % roll, pitch에 fft 적용

figure;
plot(Fs2/L2*(0:L2-1),abs(opti_fftr),"LineWidth",1.5); hold on;
plot(Fs2/L2*(0:L2-1),abs(opti_fftp),"LineWidth",1.5);

title("Complex Magnitude of fft Spectrum");
xlabel("f (Hz)"); ylabel("|fft(X)|"); legend("opti roll","opti pitch");


% --- Low-pass filter optitrack noise ---
eulDeg(:,3) = lowpass(eulDeg(:,3), 0.273, Fs2);  % roll
eulDeg(:,2) = lowpass(eulDeg(:,2), 0.191, Fs2);  % pitch



%% --- 0. Accelerometer Low-pass filter 적용 ---
% PR_#2. 앞서 Optitrack 데이터에 대한 LPF 적용등을 참조해서 가속도 센서 데이터의 진동 잡음을 줄이는 코드 작성하시오. 
% 이때 가속도계로 측정되는 로봇팔의 진동주파수도 추정해 보시오.

Fs_acc   = 60;                   % Xsens IMU Sampling Frequency
L_acc    = numel(imuMsgs);       % Length of signal
t_acc    = (0:L_acc-1) * dt;     % Time vector
f_acc    = Fs_acc/L_acc * (0:L_acc-1)';  % Frequency vector

% FFT 수행 (x, y, z 3축)
AccX_fft = fft(accX); AccY_fft = fft(accY); AccZ_fft = fft(accZ);

% 0.5~10 Hz 대역에서 진동피크 찾기
bandMask = (f_acc > 0.5) & (f_acc < 10);   % ★ 상한 10 Hz 로 확대
if ~any(bandMask)
    error('선택한 주파수 구간(0.5–10 Hz)에 데이터가 없습니다. bandMask 확인!');
end

[~,ix] = max(abs(AccX_fft(bandMask)));
[~,iy] = max(abs(AccY_fft(bandMask)));
[~,iz] = max(abs(AccZ_fft(bandMask)));

fCandidates = f_acc(bandMask);      % 공통 후보 벡터
f_vib_x = fCandidates(ix);
f_vib_y = fCandidates(iy);
f_vib_z = fCandidates(iz);

fprintf('[Accel FFT] vib peaks  ax %.2f Hz | ay %.2f Hz | az %.2f Hz\n', ...
        f_vib_x, f_vib_y, f_vib_z);


% cut-off frequncy = 가장 높은 피크 × 1.10(10 % 여유)
fc_acc = 1.10 * max([f_vib_x, f_vib_y, f_vib_z]);
fprintf('[Accel LPF] fc = %.2f Hz 적용 (3축 공통)\n', fc_acc);

% --- Low-pass filter accelerometer noise ---
accX = lowpass(accX, fc_acc, Fs_acc);
accY = lowpass(accY, fc_acc, Fs_acc);
accZ = lowpass(accZ, fc_acc, Fs_acc);

% LPF 전·후 비교 그래프
figure('Name','Accel LPF effect','NumberTitle','off');
tiledlayout(3,1,"TileSpacing","compact","Padding","compact");
nexttile; plot(t_acc, imuMat(:,1),'k--', t_acc, accX,'r'); grid on;
ylabel('a_x'); title('a_x : raw (--) vs LPF (-)');
nexttile; plot(t_acc, imuMat(:,2),'k--', t_acc, accY,'g'); grid on;
ylabel('a_y');
nexttile; plot(t_acc, imuMat(:,3),'k--', t_acc, accZ,'b'); grid on;
ylabel('a_z'); xlabel('Time (s)');
legend('raw','LPF','Location','best');



%% --- 1. Accelerometer-based attitude estimation ---
% PR_#3. 가속도계 데이터를 이용한 Roll(φ_acc)·Pitch(θ_acc) 계산 알고리즘 작성.
% φ =  atan2( a_y , a_z )
% θ = −atan2( a_x , √(a_y² + a_z²) )
% 모든 a 는 이미 LPF로 진동/노이즈 제거된 값이다

g_est = mean( sqrt(accX.^2 + accY.^2 + accZ.^2) );   % ≈ 9.8 m/s² (참고용)

phi_acc   = atan2(  accY ,  accZ );                           % rad
theta_acc = -atan2( accX , sqrt(accY.^2 + accZ.^2) );         % rad

phi_acc_deg   = rad2deg(phi_acc);      % deg 단위
theta_acc_deg = rad2deg(theta_acc);    % deg 단위

% 결과 저장 (후속 단계에서 CF, 그래프 등 사용할 수 있도록)
euler_acc = [phi_acc_deg , theta_acc_deg];   % N×2 matrix  [φ_acc  θ_acc]

% 가속도 기반 Roll/Pitch 시각화 ----------------------------------
figure('Name','Accel-based Attitude','NumberTitle','off');

% Roll subplot
subplot(2,1,1);
plot(T, phi_acc_deg, 'r');
grid on;
ylabel('\phi_{acc} (deg)');
title('Accelerometer-based Roll');

% Pitch subplot
subplot(2,1,2);
plot(T, theta_acc_deg, 'b');
grid on;
ylabel('\theta_{acc} (deg)');
xlabel('Time (s)');
title('Accelerometer-based Pitch');



%% --- 1-1. 자세 초기치는 주어진 Optitrack(reference) data 기준으로 맞춤 ---
% PR_#3.1 가속도계 기반 자세각의 초기치를 앞서 optitrack 기준값으로 사용해 볼 것.
% 가속도 기반 Roll/Pitch(φ_acc, θ_acc)는 센서 장착 각도·초기 보정 오차
% 때문에 일정한 오프셋이 존재할 수 있다. 따라서 첫 샘플에서의
% Optitrack 참값과 일치하도록 상수 보정을 적용한다.

initN = round(1 / dt);          % ≈ 60개 (1초)

% Optitrack(클린) 첫 샘플 [deg]
phi_ref0   = mean( eulDeg(1:initN,3) );      % Roll  (φ_ref[0])
theta_ref0 = mean( eulDeg(1:initN,2) );      % Pitch (θ_ref[0])

% 가속도 기반 첫 샘플 [deg]
phi_acc0   = mean( phi_acc_deg(1:initN) );
theta_acc0 = mean( theta_acc_deg(1:initN) );

% 오프셋 보정값
dPhi   = phi_ref0   - phi_acc0;
dTheta = theta_ref0 - theta_acc0;

% 전체 시계열에 상수 오프셋 적용
phi_acc_deg   = phi_acc_deg   + dPhi;
theta_acc_deg = theta_acc_deg + dTheta;

% euler_acc 행렬 업데이트 (후속 단계·상보필터에서 사용)
euler_acc = [phi_acc_deg , theta_acc_deg];

fprintf('[Init align]  φ_offset = %.4f deg,  θ_offset = %.4f deg  (applied to acc-based angles)\n', ...
        dPhi, dTheta);



%% --- 2. Gyro-based attitude integration ---
% PR_#4 자이로 기반 자세각 계산 알고리즘 작성.
% 드리프트 효과를 보여주기 위해
% rawGyro  : 바이어스 미보정  (p,q,r)
% corGyro  : 바이어스 보정    (p_bcorr,q_bcorr,r_bcorr)
% 두 경우를 모두 적분하여 비교.

% ── 준비 : 초기값(φ0, θ0)은 Optitrack 클린 참값 첫 샘플 사용 ──
phi0   = deg2rad( eulDeg(1,3) );   % rad
theta0 = deg2rad( eulDeg(1,2) );   % rad

% 배열 초기화
phi_raw   = zeros(N,1);  theta_raw   = zeros(N,1);
phi_corr  = zeros(N,1);  theta_corr  = zeros(N,1);
phi_raw(1)=phi0;         theta_raw(1)=theta0;
phi_corr(1)=phi0;        theta_corr(1)=theta0;

% 변환 행렬 M(φ,θ) 함수 (body-rate→Euler-rate)
M = @(phi,theta) [ ...
      1              tan(theta)*sin(phi)      tan(theta)*cos(phi);
      0                     cos(phi)                -sin(phi)     ] ;

% 적분 루프 (Euler forward)
for k = 1:N-1
    % ---------- (a) 바이어스 미보정 ----------
    Graw = [gyroX(k); gyroY(k); gyroZ(k)];      % deg/s
    Graw = deg2rad(Graw);                       % → rad/s
    eDot = M(phi_raw(k),theta_raw(k)) * Graw(1:3);  % φ̇ θ̇
    phi_raw(k+1)   = phi_raw(k)   + eDot(1)*dt;
    theta_raw(k+1) = theta_raw(k) + eDot(2)*dt;

    % ---------- (b) 바이어스 보정 ----------
    Gcor = [gyroX_c(k); gyroY_c(k); gyroZ_c(k)];
    Gcor = deg2rad(Gcor);
    eDotC = M(phi_corr(k),theta_corr(k)) * Gcor;
    phi_corr(k+1)   = phi_corr(k)   + eDotC(1)*dt;
    theta_corr(k+1) = theta_corr(k) + eDotC(2)*dt;
end

% deg 단위 변환
phi_raw_deg   = rad2deg(phi_raw);
theta_raw_deg = rad2deg(theta_raw);
phi_corr_deg  = rad2deg(phi_corr);
theta_corr_deg= rad2deg(theta_corr);

% ── 시각화 : Roll / Pitch ────────────────────────────────────────────
figure('Name','Gyro-only attitude (bias effect)','NumberTitle','off');
subplot(2,1,1);
plot(T, phi_raw_deg,'r',  T, phi_corr_deg,'k');
grid on; ylabel('\phi (deg)');
title('Roll from gyro integration  – red: raw, black: bias-removed');
legend('raw-integrated','bias-removed','Location','best');

subplot(2,1,2);
plot(T, theta_raw_deg,'r',  T, theta_corr_deg,'k');
grid on; ylabel('\theta (deg)'); xlabel('Time (s)');
title('Pitch from gyro integration');


%% --- 3. Complementary filter (ACC+GYRO) ---
% PR_#5 상보필터 이용한 센서 융합 자세각 계산 알고리즘 작성.
% alpha 크기 자율적으로 튜닝

%% --- 3. Complementary filter (ACC + GYRO) -----------------------------
%  α : Gyro 비중(고주파),  (1–α) : Acc 비중(저주파)
alpha = 0.98;        % 0.95~0.99 사이에서 바꿔가며 튜닝해보세요

% ── 초기값 : 이미 Optitrack에 정렬한 acc-기반 Roll/Pitch로 시작 ──
phi_CF   = zeros(N,1);  theta_CF   = zeros(N,1);
phi_CF(1)   = phi_acc_deg(1);
theta_CF(1) = theta_acc_deg(1);

for k = 2:N
    % Gyro 변화량(Δφ_gyro·Δθ_gyro) : bias-removed 적분값 차분으로 근사
    dPhi_gyro   =  phi_corr_deg(k)   - phi_corr_deg(k-1); % deg
    dTheta_gyro =  theta_corr_deg(k) - theta_corr_deg(k-1);

    % 예측(gyro) + 보정(acc)
    phi_CF(k)   = alpha*(phi_CF(k-1)   + dPhi_gyro)   + (1-alpha)*phi_acc_deg(k);
    theta_CF(k) = alpha*(theta_CF(k-1) + dTheta_gyro) + (1-alpha)*theta_acc_deg(k);
end

% ── 결과 비교 그래프 (Optitrack vs Gyro vs Acc vs CF) ───────────────
figure('Name','Complementary Filter Result','NumberTitle','off');
subplot(2,1,1)
plot(T_2, eulDeg(:,3),'k',  ...
     T, phi_corr_deg,'r--', ...
     T, phi_acc_deg,'g:', ...
     T, phi_CF,'b','LineWidth',1.1);
grid on; ylabel('\phi (deg)');
title(sprintf('Roll – Opti(black)  Gyro(red)  Acc(green)  CF(blue, \\alpha=%.2f)',alpha));
legend('Optitrack','Gyro-only','Acc-only','Complementary');

subplot(2,1,2)
plot(T_2, eulDeg(:,2),'k', ...
     T, theta_corr_deg,'r--', ...
     T, theta_acc_deg,'g:', ...
     T, theta_CF,'b','LineWidth',1.1);
grid on; ylabel('\theta (deg)'); xlabel('Time (s)');
title('Pitch – Opti / Gyro / Acc / CF');

%% --- 최적의 alpha 찾기 ---
% RMSE(Root Mean Square Error)로 확인
alphas  = 0.90:0.01:0.99;           % 후보 α 범위
rmsePhi = zeros(size(alphas));      % Roll RMSE 저장
rmseThe = zeros(size(alphas));      % Pitch RMSE 저장

for i = 1:numel(alphas)
    a = alphas(i);

    % --- 간단히 1-차 CF 계산 (벡터화) ---
    phi_cf   = zeros(N,1);  theta_cf   = zeros(N,1);
    phi_cf(1)=phi_acc_deg(1);  theta_cf(1)=theta_acc_deg(1);

    for k = 2:N
        dPhi   = phi_corr_deg(k)   - phi_corr_deg(k-1);
        dTheta = theta_corr_deg(k) - theta_corr_deg(k-1);
        phi_cf(k)   = a*(phi_cf(k-1)+dPhi)   + (1-a)*phi_acc_deg(k);
        theta_cf(k) = a*(theta_cf(k-1)+dTheta) + (1-a)*theta_acc_deg(k);
    end

    % ── Optitrack 과 길이 맞추기 (보간) ──
    phi_ref   = interp1(T_2, eulDeg(:,3), T, 'linear','extrap');
    theta_ref = interp1(T_2, eulDeg(:,2), T, 'linear','extrap');

    rmsePhi(i) = sqrt( mean( (phi_cf   - phi_ref  ).^2 ) );
    rmseThe(i) = sqrt( mean( (theta_cf - theta_ref).^2 ) );
end

[~,bestIdx] = min(rmsePhi + rmseThe);   % Roll+Pitch 합산 최소
bestAlpha  = alphas(bestIdx);
fprintf('최적 α ≈ %.2f  (RMSE_Roll %.2f°,  RMSE_Pitch %.2f°)\n', ...
        bestAlpha, rmsePhi(bestIdx), rmseThe(bestIdx));

 

%% --- 4. Plot Roll Estimation ---
% PR_#6 그래프 분석 및 토의
figure('Name','PR#6  Roll','NumberTitle','off');

plot(T_2, eulDeg(:,3), 'k', ...      % Optitrack
     T,   phi_CF, 'r', 'LineWidth', 1);  % Complementary Filter
grid on; xlabel('Time (s)'); ylabel('\phi (deg)');
title('Roll  –  Opti  /  CF');
legend('Optitrack','CF','Location','SouthWest');

phi_ref = interp1(T_2, eulDeg(:,3), T);
fprintf('Roll  RMSE  CF %.2f°\n', rms(phi_CF - phi_ref));



%% --- 5. Plot Pitch Estimation ---
% PR_#7 그래프 분석 및 토의
figure('Name','PR#7  Pitch','NumberTitle','off');

plot(T_2, eulDeg(:,2), 'k', ...
     T,   theta_CF, 'r', 'LineWidth', 1);  % Complementary Filter
grid on; xlabel('Time (s)'); ylabel('\theta (deg)');
title('Pitch  –  Opti  /  CF');
legend('Optitrack','CF','Location','SouthWest');

theta_ref = interp1(T_2, eulDeg(:,2), T);
fprintf('Pitch RMSE  CF %.2f°\n', rms(theta_CF - theta_ref));

