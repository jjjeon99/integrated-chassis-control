# 202524357-전재진 ICC 제어기 설계 보고서

**과목**: 자동제어 - 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

---

## 1. 설계 개요

본 과제의 목표는 통합 섀시 제어(Integrated Chassis Control, ICC)를 이용하여 조향, 제동, 현가 actuator를 동시에 배분하고, A1/A3/A4/A7/B1/D1 시나리오에서 baseline 대비 안정성과 제동 성능을 개선하는 것이다. 검증 plant는 14DOF 차량 모델이지만, 제어기 설계는 실시간 구현성과 해석 가능성을 위해 bicycle model 기반 yaw-rate 추종, slip-angle limiter, ABS slip-ratio limiter, skyhook CDC로 단순화하였다.

선택한 제어기법은 gain-scheduled PI/PD 계열 제어와 rule-based ESC/ABS allocation이다. LQR이나 MPC처럼 모델 의존도가 높은 기법도 가능하지만, 본 과제의 runner는 여러 시나리오와 강한 비선형 tire saturation을 포함하므로, 각 KPI에 대응하는 물리량을 직접 제한하는 구조가 더 안정적이라고 판단하였다. 특히 yaw rate, side-slip, LTR, wheel slip은 차량 안정성 평가에서 직접 쓰이는 상태량이므로, 이들을 명시적으로 제한하는 방식이 튜닝과 해석에 유리했다.

각 제어기 요약은 다음과 같다.

- **ctrl_lateral**: 속도 스케줄링 PI 기반 AFS yaw-rate 추종, bicycle-model feedforward, slip-angle 기반 ESC yaw moment 보조
- **ctrl_longitudinal**: PI 속도 추종 + wheel-slip 기반 ABS relief 제어
- **ctrl_vertical**: hybrid skyhook/groundhook CDC + anti-roll damping override
- **ctrl_coordinator**: longitudinal brake 60:40 배분, ESC yaw moment를 좌우 brake differential로 변환, straight-brake ABS relief 통과

---

## 2. 수학적 모델링

### 2.1 사용한 plant 단순화

제어 설계에는 선형 bicycle model을 사용하였다. 실제 검증은 roll, pitch, wheel rotational dynamics, suspension dynamics를 포함한 14DOF plant에서 수행되지만, lateral controller의 기본 yaw-rate tracking gain은 bicycle model의 $v_y-r$ 동역학으로 해석하였다. longitudinal ABS는 단일 wheel rotational dynamics와 slip ratio 정의를 기준으로 설계하였다. vertical CDC는 quarter-car skyhook 해석을 기반으로 하되, 네 코너의 roll/pitch modal velocity를 추가로 사용하였다.

### 2.2 Bicycle model

상태, 입력, 출력은 다음과 같이 두었다.

$$x = [v_y, r]^T, \quad u = \delta, \quad y = r$$

선형 tire 영역에서 전후륜 cornering stiffness를 $C_f, C_r$라 하면,

$$\dot{x} = Ax + Bu$$

이고,

$$
\dot{v}_y =
-\frac{C_f + C_r}{mV_x}v_y
+ \left(\frac{l_r C_r - l_f C_f}{mV_x} - V_x\right)r
+ \frac{C_f}{m}\delta
$$

$$
\dot{r} =
\frac{l_r C_r - l_f C_f}{I_z V_x}v_y
- \frac{l_f^2 C_f + l_r^2 C_r}{I_z V_x}r
+ \frac{l_f C_f}{I_z}\delta
$$

따라서 조향 입력에 대한 yaw-rate 응답은 속도 $V_x$에 따라 크게 달라진다. 고속에서는 작은 조향 입력에도 yaw response가 커지므로, controller gain은 고속에서 과도하게 증가하지 않도록 제한해야 한다.

### 2.3 Wheel slip model

ABS 제어는 다음 slip ratio 정의를 사용하였다.

$$
\kappa_i = \frac{\omega_i r_w - V_{x,i}}{\max(|V_{x,i}|, \epsilon)}
$$

제동 중에는 wheel speed가 감소하므로 $\kappa$가 음수로 나타날 수 있다. runner에서 전달되는 부호가 모델/로그 convention에 따라 달라질 수 있으므로, 제어기 내부에서는

$$
\kappa_{brake,i} = |\kappa_i|
$$

를 사용하였다. 목표 slip은 $0.12 \sim 0.14$ 근방으로 두고, 목표보다 큰 경우 controller brake torque를 음수로 요청하여 scenario brake torque를 상쇄한다.

### 2.4 가정과 한계

- 제어 설계 단계에서는 종방향 속도 $V_x$를 quasi-static parameter로 보고 lateral/longitudinal/vertical dynamics를 분리하였다.
- Tire force는 소슬립 영역에서 선형으로 근사하였으나, 실제 14DOF plant에서는 combined-slip saturation이 발생한다.
- 경로 오차(lateral deviation)는 현재 `ctrl_lateral` 함수 입력에 직접 들어오지 않는다. 따라서 yaw-rate tracking만으로는 A1/D1의 lateralDevMax를 완전히 줄이는 데 한계가 있다.
- ABS는 scenario brake command 위에 controller brake torque가 더해지는 구조를 이용한다. 따라서 controller의 음수 brake torque는 master-cylinder pressure relief로 해석된다.

---

## 3. 제어기 설계

### 3.1 ctrl_lateral - AFS + ESC

**설계 목표**

- A3 step steer에서 yaw-rate overshoot 및 settling 개선
- A1/D1 double lane change에서 side-slip과 LTR 억제
- A4 steady-state circular에서 understeer gradient와 side-slip 악화 방지
- A7 brake-in-turn에서 slip-angle과 LTR 안정화

**AFS 구조**

AFS는 yaw-rate error에 대한 PI controller와 feedforward를 결합하였다.

$$
e_r = r_{ref} - r
$$

$$
\delta_{AFS} =
K_p(V_x)e_r + K_i(V_x)\int e_r\,dt + \delta_{ff}
$$

초기에는 raw finite-difference derivative term도 사용했으나, 고속 step steer에서 chattering과 비현실적인 rise time이 발생하여 최종 설계에서는

```matlab
kdEff = 0;
```

으로 비활성화하였다. D항을 제거한 대신 고속 gain scheduling과 feedforward로 phase lag를 줄였다.

최종 scheduling은 다음과 같다.

```matlab
speedAtten = local_sat((vxAbs - 5.0) / 20.0, 0, 1);
kpSched = 1.0 - 0.1 * speedAtten;
kiSched = 1.0 - 0.9 * speedAtten;

kpEff = kp * kpSched;
kiEff = ki * kiSched;
kdEff = 0;
```

고속에서 $K_p$는 최대 10%만 줄여 조향 반응성을 유지하고, $K_i$는 최대 90% 줄여 accumulated yaw error가 복귀 구간에서 잔진동을 만들지 않도록 하였다. 적분 상태도 다음과 같이 강하게 제한했다.

```matlab
intEffMax = min(0.05 * intMax, intMax * kiSched);
```

Feedforward는 bicycle model의 저속 근사식 $\delta \approx Lr/V_x$를 사용하였다.

```matlab
wheelbaseFF = 2.7;
steerFF = 1.05 * wheelbaseFF * yawRateRefSafe / max(vxAbs, 1.0);
```

이 항은 yaw-rate error가 커진 뒤에 반응하는 feedback의 phase lag를 줄이기 위한 것이다.

**ESC 구조**

Side-slip angle이 임계값을 넘으면 beta-limiter yaw moment를 사용한다.

$$
M_z = M_{track} + M_{\beta}
$$

```matlab
betaThreshold = min(deg2rad(3.0), 0.75 * slipHardLimit);
betaExcess = max(abs(slipAngle) - betaThreshold, 0);
mzSlip = speedBlend * yawMomentLimit * local_sat(betaNorm, -1, 1);
```

슬립이 아직 임계값 이하이지만 yaw-rate error가 큰 경우에는 understeer 보조용 tracking yaw moment를 사용한다.

```matlab
if betaExcess > 0
    yawMomentCmd = mzTrack + mzSlip;
else
    if abs(yawNorm) > 0.25
        yawMomentCmd = 0.85 * mzTrack;
    else
        yawMomentCmd = 0;
    end
end
```

이는 전륜 조향을 더 주는 방식이 tire saturation에 막힐 때, brake differential을 이용해 yaw moment를 추가로 만드는 목적이다.

### 3.2 ctrl_longitudinal - 속도 추종 + ABS

종방향 제어기는 PI 속도 추종을 기본으로 하되, B1 straight brake에서는 ABS가 핵심이다. 외부 scenario brake가 들어오는 동안 제어기가 양의 drive force를 내지 않도록 제한하였다.

```matlab
externalBrakeActive = (ax < -0.5) && (vx > 1.0);
if externalBrakeActive
    fxUnsat = min(fxUnsat, 0);
end
```

ABS는 wheel slip magnitude를 사용한다.

```matlab
brakeSlip = abs(ctrlState.wheelSlip(:));
```

최종 ABS 목표 slip은 $0.13$으로 두었다. 목표보다 slip이 크면 controller가 음수 brake assist ratio를 출력하여 scenario brake torque를 상쇄한다.

```matlab
slipTarget = 0.13;
slipError = brakeSlip - slipTarget;
wheelAssistTarget(releaseMask) = -20.0 * slipError(releaseMask);
wheelAssistTarget(~releaseMask) = min(0.0, prevAbsCmd(~releaseMask) + 1.5 * dt);
wheelAssistTarget = local_sat(wheelAssistTarget, -1.0, 0.0);
```

중요한 설계 선택은 positive brake assist를 제거한 것이다. ABS가 slip을 제어하는 동안 controller가 driver보다 더 brake를 밟는 일이 없도록 상한을 0으로 제한하였다.

### 3.3 ctrl_vertical - CDC

CDC는 clipped skyhook을 기본으로 하였다.

```matlab
cSky = skyGain * abs(zsDot(i)) / max(abs(relVelEff), 0.05);
if zsDot(i) * relVel(i) > 0
    cCmd = cSky;
else
    cCmd = cMin;
end
```

여기에 wheel-hop 억제를 위한 groundhook 성분과 braking pitch support를 추가하였다. A1/D1처럼 급격한 lane change에서는 roll-rate가 빠르게 커지므로, skyhook 조건에 걸려 damping이 낮아지는 것을 막기 위해 anti-roll damping floor를 추가하였다.

```matlab
rollSupport = local_sat(abs(rollVel) / 0.16, 0, 1);
rollFloor = cMin + 0.42 * (cMax - cMin) * rollSupport;
cCmd = max(cCmd, rollFloor);
```

Damping command는 상승 시 빠르게, 감소 시 천천히 변하도록 비대칭 smoothing을 적용했다.

```matlab
alphaRise = local_sat(dt / 0.004, 0, 1);
alphaFall = local_sat(dt / 0.030, 0, 1);
```

### 3.4 ctrl_coordinator - Actuator allocation

Coordinator는 네 가지 역할을 한다.

1. AFS steering command saturation
2. CDC damping coefficient clipping
3. Longitudinal brake torque 60:40 front/rear allocation
4. ESC yaw moment를 좌우 differential brake torque로 변환

Yaw moment allocation은 다음 관계를 따른다.

$$
\Delta T_f = -2 \frac{\lambda_f M_z r_w}{t_f}, \quad
\Delta T_r = -2 \frac{(1-\lambda_f)M_z r_w}{t_r}
$$

여기서 $\lambda_f=0.6$으로 두었다. Plant sign convention에 맞춰 positive yaw moment는 left-side brake torque가 right-side보다 커지는 방향으로 분배하였다.

Straight brake ABS에서는 controller brake torque가 runner에서 scenario brake torque에 더해진다. 따라서 음수 controller brake torque는 실제로 scenario brake torque를 줄이는 relief request가 된다.

```matlab
if isStraightBrake
    baseBrake = local_sat(baseBrake, -0.85 * maxBrakeTrq, maxBrakeTrq);
else
    baseBrake = local_sat(baseBrake, 0, maxBrakeTrq);
end
```

---

## 4. 시뮬레이션 결과

최신 `grade_report.json` 기준 자동 채점 결과는 다음과 같다.

- 정량 점수: **50.8109 / 70**
- 비율: **72.59%**
- 런타임 에러: 없음
- 감점: 없음

### 4.1 P1 시나리오 KPI 요약

| 시나리오 | KPI | OFF | ON | 목표 | 점수 |
|---|---:|---:|---:|---:|---:|
| A3 Step Steer | yawRateOvershoot | 2.6997 | 3.3106 | 10.0000 | 0.00 / 4 |
| A3 Step Steer | yawRateRiseTime [s] | - | 0.0700 | 0.3000 | 4.00 / 4 |
| A3 Step Steer | yawRateSettling [s] | - | 1.2360 | 0.8000 | 1.82 / 4 |
| A1 DLC | sideSlipMax [deg] | 3.0154 | 2.7419 | 3.0000 | 6.00 / 6 |
| A1 DLC | LTR_max | 0.8635 | 0.7596 | 0.6000 | 3.67 / 5 |
| A1 DLC | lateralDevMax [m] | 1.8270 | 1.8718 | 0.7000 | 0.00 / 4 |
| A4 SS Circular | understeerGradient | - | 0.00078 | 0.0030 | 5.00 / 5 |
| A4 SS Circular | sideSlipMax [deg] | 1.1839 | 1.1761 | 2.0000 | 5.00 / 5 |
| A7 Brake-in-Turn | sideSlipMax [deg] | 30.4776 | 2.1493 | 5.0000 | 8.00 / 8 |
| A7 Brake-in-Turn | LTR_max | 0.6808 | 0.3626 | 0.7000 | 7.00 / 7 |
| B1 Straight Brake | stoppingDistance [m] | 72.2992 | 68.6208 | 65.5000 | 4.52 / 5 |
| B1 Straight Brake | absSlipRMS | - | 0.2401 | 0.1000 | 0.33 / 5 |
| D1 DLC+Brake | sideSlipMax [deg] | 4.9057 | 3.2589 | 4.0000 | 4.00 / 4 |
| D1 DLC+Brake | LTR_max | 0.8635 | 0.7596 | 0.6000 | 1.47 / 2 |
| D1 DLC+Brake | lateralDevMax [m] | 1.8270 | 1.8718 | 1.0000 | 0.00 / 2 |

### 4.2 핵심 plot 생성 방법

본 보고서에는 코드 제출 환경에서 그림 파일을 직접 포함하지 않았지만, 다음 명령으로 A1 trajectory와 yaw rate plot을 생성할 수 있다.

```matlab
[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);

figure;
plot(r_off.x_pos, r_off.y_pos, 'r--'); hold on;
plot(r_on.x_pos, r_on.y_pos, 'b-');
plot(r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');
xlabel('x [m]'); ylabel('y [m]');
legend('off','on','reference'); axis equal; grid on;
saveas(gcf, 'docs/figures/a1_trajectory.png');

figure;
plot(r_on.t, r_on.yawRateRef, 'k:'); hold on;
plot(r_off.t, r_off.yawRate, 'r--');
plot(r_on.t, r_on.yawRate, 'b-');
xlabel('time [s]'); ylabel('yaw rate [rad/s]');
legend('reference','off','on'); grid on;
saveas(gcf, 'docs/figures/a1_yawrate.png');
```

### 4.3 Deep dive - A7 Brake-in-Turn

A7은 가장 성공적으로 안정화된 시나리오이다. Baseline에서는 제동 중 선회가 겹치며 side-slip angle이 약 30 deg 이상으로 커졌고, 이는 사실상 spin-out에 가까운 거동이다. 본 설계에서는 slip-angle limiter가 작동하여 yaw moment를 만들고, coordinator가 이를 좌우 differential brake torque로 바꾸어 차량의 yaw와 side-slip을 억제하였다.

결과적으로 A7은 다음 KPI에서 만점을 얻었다.

- sideSlipMax: 2.1493 deg, target 5 deg 이하
- LTR_max: 0.3626, target 0.7 이하

이는 ESC yaw moment와 CDC roll damping이 동시에 작동하여 brake-in-turn 상태에서 차체 slip과 load transfer를 모두 억제했기 때문이라고 해석된다.

---

## 5. 분석과 한계

### 5.1 가장 성공적이었던 시나리오

가장 성공적인 시나리오는 A7 Brake-in-Turn이다. A7은 조향과 제동이 동시에 들어가므로 side-slip과 LTR이 모두 커지기 쉽다. 본 제어기에서는 slip-angle limiter, yaw moment allocation, CDC anti-roll damping이 모두 안정성 방향으로 작동하였다. 특히 sideSlipMax가 baseline 대비 크게 감소하여 spin-out을 방지하였다.

A4 steady-state circular도 안정적으로 통과하였다. 이는 steady/benign corner guard가 과도한 AFS 개입을 줄이고, ESC가 필요할 때만 yaw moment를 발생시키도록 제한했기 때문이다.

### 5.2 가장 부족했던 시나리오

가장 큰 한계는 A1/D1의 lateralDevMax이다. 현재 `ctrl_lateral`은 yawRateRef, yawRate, slipAngle, vx만 입력으로 받으며, 실제 path lateral error는 입력으로 받지 않는다. 따라서 yaw rate tracking은 좋아져도 차량이 reference path에서 얼마나 옆으로 밀렸는지를 직접 알 수 없다. 최신 결과에서도 A1/D1 lateralDevMax는 약 1.87 m로 남아 있다.

B1 ABS도 아직 한계가 남아 있다. stoppingDistance는 baseline 72.3 m에서 68.6 m로 줄었지만, absSlipRMS는 0.24 수준으로 target 0.10에 미치지 못했다. 원인은 다음과 같이 추정된다.

- scenario brake torque가 강해 controller relief가 완전히 lock/unlock oscillation을 제거하지 못함
- wheel slip feedback이 one-step delayed cache로 들어와 ABS phase lag가 존재함
- controller output이 scenario brake command에 더해지는 구조라 실제 hydraulic pressure dynamics를 직접 제어하지 못함

### 5.3 만약 더 시간이 있었다면

1. **Lateral path error feedback 추가**
   `ctrl_lateral` 입력에 lateral deviation 또는 preview path error를 추가하면 A1/D1의 lateralDevMax를 직접 줄일 수 있다. 현재 yaw-rate tracking만으로는 path tracking 오차를 완전히 보상하기 어렵다.

2. **ABS hydraulic pressure state 도입**
   현재 ABS는 brakeAssist ratio를 직접 계산하지만, 실제 hydraulic pressure의 rate limit과 hold/decrease/increase mode를 별도 상태로 두면 slip RMS를 더 부드럽게 줄일 수 있다.

3. **WLS allocation**
   yaw moment, total brake force, per-wheel slip relief를 동시에 만족하는 weighted least-squares allocator를 구현하면 ESC와 ABS가 서로 충돌하는 구간에서 더 안정적인 actuator command를 만들 수 있다.

4. **Gain scheduling table화**
   현재 scheduling은 affine function 위주이다. A3/A1/A7/B1처럼 동역학이 다른 시나리오를 하나의 함수로 커버하려면 speed, brake activity, slip level에 따른 2D scheduling map이 더 적합하다.

---

## 6. 참고문헌

[1] ISO 3888-1:2018, *Passenger cars - Test track for a severe lane-change manoeuvre*.
[2] ISO 4138:2021, *Passenger cars - Steady-state circular driving behaviour*.
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012.
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008.
[5] T. D. Gillespie, *Fundamentals of Vehicle Dynamics*, SAE International, 1992.

---

## 부록 A - 사용한 AI 도구

`student_info.m`의 `ai_usage` 항목과 일치하게 Codex를 사용하였다. Codex는 제어기 구조 정리, MATLAB 코드 수정, gain tuning 후보 제안, 보고서 초안 작성에 사용되었다. 최종 설계 판단은 benchmark 결과를 확인하며 반복적으로 조정하였다.

---

## 부록 B - 주요 코드 변경 요약

### ctrl_lateral.m

```matlab
kpSched = 1.0 - 0.1 * speedAtten;
kiSched = 1.0 - 0.9 * speedAtten;
kdEff = 0;
steerFF = 1.05 * wheelbaseFF * yawRateRefSafe / max(vxAbs, 1.0);
yawMomentCmd = 0.85 * mzTrack;  % beta safe, yaw error large
```

### ctrl_longitudinal.m

```matlab
brakeSlip = abs(ctrlState.wheelSlip(:));
slipTarget = 0.13;
wheelAssistTarget(releaseMask) = -20.0 * slipError(releaseMask);
wheelAssistTarget(~releaseMask) = min(0.0, prevAbsCmd(~releaseMask) + 1.5 * dt);
```

### ctrl_vertical.m

```matlab
rollSupport = local_sat(abs(rollVel) / 0.16, 0, 1);
rollFloor = cMin + 0.42 * (cMax - cMin) * rollSupport;
cCmd = max(cCmd, rollFloor);
```

### ctrl_coordinator.m

```matlab
brakeAssistWheelRatio = local_sat(brakeAssistWheelRatio, -4.0, 1);
baseBrake = local_sat(baseBrake, -0.85 * maxBrakeTrq, maxBrakeTrq);
```
