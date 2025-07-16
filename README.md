# 드론 자세 추정 프로젝트 (IMU & Complementary Filter)

건국대학교 '신호 및 시스템' 과목의 텀 프로젝트입니다.

본 프로젝트는 ROS bag 파일로 수집된 IMU(Xsens) 센서와 OptiTrack 시스템 데이터를 사용하여 드론의 자세(Roll, Pitch)를 추정하는 것을 목표로 합니다. 각 센서 데이터의 특성을 분석하고, 상보 필터(Complementary Filter)를 이용한 센서 퓨전 기법을 구현하여 그 성능을 검증합니다.

<br>

## 1. 프로젝트 목표 및 전체 흐름

ROS bag 파일에서 IMU(가속도, 자이로)와 OptiTrack(기준값) 데이터를 추출하고 전처리합니다. 이후 가속도계와 자이로스코프의 단점을 보완하는 상보 필터를 설계하여 두 센서의 장점을 융합하고, 이를 통해 안정적이고 정확한 자세(Roll, Pitch)를 추정하는 것이 최종 목표입니다.

![Flowchart](https://raw.githubusercontent.com/imhyeonwoo/imhyeonwoo/main/assests/Konkuk/Signal%26System/flowchart.png)

<br>

## 2. 실험 환경

본 프로젝트에 사용된 실험 환경입니다. UGV, 로봇팔, OptiTrack 시스템을 이용하여 드론의 3D 모션을 모사하고 데이터를 수집했습니다.

| 실험 환경 1 | 실험 환경 2 |
| :---: | :---: |
| ![Environment1](https://raw.githubusercontent.com/imhyeonwoo/imhyeonwoo/main/assests/Konkuk/Signal%26System/Environment1.png) | ![Environment2](https://raw.githubusercontent.com/imhyeonwoo/imhyeonwoo/main/assests/Konkuk/Signal%26System/Environment2.png) |

<br>

## 3. 구현 과정 및 핵심 알고리즘

### 3.1. 데이터 추출 및 유효성 검사

-   **`packet_student.m`** 스크립트를 사용하여 `signal_exp_data.bag` 파일에서 필요한 IMU, OptiTrack 토픽을 추출하고 `packet_data.mat` 파일로 저장합니다.
-   추출된 데이터의 유효성을 검증하기 위해 정지 구간에서 가속도 크기가 중력가속도(9.8m/s²)에 근사하는지, 자이로 크기가 0 deg/sec에 근사하는지, 쿼터니언의 노름(norm)이 1인지 확인합니다.

### 3.2. 자이로스코프 Bias 제거

-   자이로 센서는 정지 상태에서도 미세한 출력값(Bias)을 가지며, 이를 적분하면 시간이 지남에 따라 오차가 누적(Drift)됩니다.
-   초기 30초의 정지 구간 데이터를 평균 내어 Bias를 계산하고, 전체 자이로 데이터에서 이 값을 빼주어 Drift 현상을 최소화합니다.

### 3.3. 주파수 분석 및 노이즈 필터링 (FFT & LPF)

-   가속도계와 OptiTrack 데이터에는 로봇팔의 미세 진동으로 인한 고주파 노이즈가 포함되어 있습니다.
-   FFT(고속 푸리에 변환)를 통해 노이즈의 주파수 대역을 분석하고, LPF(저주파 통과 필터)를 적용하여 유효 신호는 유지하면서 노이즈 성분만 효과적으로 제거합니다.
-   **가속도계 진동 주파수**: 약 0.6 Hz
-   **OptiTrack 진동 주파수**: 약 0.2 Hz

### 3.4. 상보 필터 (Complementary Filter) 구현

-   **가속도계**: 저주파(느린 움직임, 정지 상태)에서는 중력을 기준으로 비교적 정확한 기울기를 측정하지만, 동적 환경(외부 가속, 진동)에서는 노이즈에 취약합니다.
-   **자이로스코프**: 고주파(빠른 움직임)의 각도 변화를 잘 측정하지만, Bias로 인해 시간이 지날수록 오차가 누적됩니다.

상보 필터는 두 센서의 장점을 융합하는 기법으로, 아래 공식과 같이 구현됩니다.
`자세(t) = α * (이전 자세 + 자이로 변화량) + (1 - α) * (가속도계 기반 자세)`

-   **최적의 α 찾기**: 기준값(OptiTrack)과의 RMSE(평균 제곱근 오차)를 최소화하는 `α` 값을 탐색한 결과, **`α = 0.98`** 에서 가장 우수한 성능을 보였습니다. 이는 빠른 움직임에 대한 자이로 데이터에 높은 신뢰도를 부여하는 것을 의미합니다.

<br>

## 4. 최종 결과

상보 필터를 적용한 Roll, Pitch 추정 결과를 기준 데이터인 OptiTrack과 비교한 그래프입니다. 자이로 단독 사용 시의 Drift와 가속도계 단독 사용 시의 노이즈 문제를 모두 보완하여, 기준값과 매우 유사한 안정적인 추정 결과를 보여줍니다.

| Roll 추정 결과 비교 | Pitch 추정 결과 비교 |
| :---: | :---: |
| ![Result1](https://github.com/imhyeonwoo/imhyeonwoo/blob/main/assests/Konkuk/Signal%26System/roll.jpg) | ![Result2](https://github.com/imhyeonwoo/imhyeonwoo/blob/main/assests/Konkuk/Signal%26System/pitch.jpg) |

-   **Roll RMSE**: Gyro `0.73°`, Acc `0.74°`, **CF `0.74°`**
-   **Pitch RMSE**: Gyro `1.44°`, Acc `1.37°`, **CF `1.34°`**

결과적으로 Complementary Filter는 두 센서의 출력을 적절히 융합하여 가장 낮은 RMSE 값을 기록하며 비교적 정확한 추정 결과를 보여주었습니다.

<br>

## 5. 실행 방법

1.  이 저장소를 로컬 컴퓨터에 복제(Clone)합니다.
    ```bash
    git clone https://github.com/imhyeonwoo/Drone-Attitude-Estimation-Project.git
    ```
2.  MATLAB에서 프로젝트 폴더를 엽니다.
3.  `signal_exp_data.bag` 파일을 프로젝트 폴더 내에 위치시킵니다. (해당 파일은 data 폴더에 있습니다.)
4.  `packet_student.m`을 실행하여 `.bag` 파일로부터 `packet_data.mat` 파일을 생성합니다.
5.  `ARS_student_V1.m`을 실행하여 모든 알고리즘을 수행하고 결과 그래프를 확인합니다.

<br>

## 6. 파일 설명

-   **`ARS_student_V1.m`**: 자세 추정 알고리즘(Bias 제거, 필터링, 상보 필터)을 구현하고 결과를 시각화하는 메인 스크립트.
-   **`packet_student.m`**: ROS bag 파일에서 데이터를 추출하고 전처리하여 `.mat` 파일로 저장하는 스크립트.

<br>

## 7. For More Detail

-   visit https://www.notion.so/Signals-and-Systems_Term-Project-2315421b258280499899feb0629cd694
