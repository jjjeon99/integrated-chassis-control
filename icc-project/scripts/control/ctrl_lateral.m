function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% 1. 내부 상태(ctrlState) 및 변수 초기화
    if nargin < 8 || ~isfinite(dt) || dt <= 0
        dt = 0.01;
    end
    if nargin < 5 || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    yawRateRef = local_safe_scalar(yawRateRef, 0);
    yawRate    = local_safe_scalar(yawRate, 0);
    slipAngle  = local_safe_scalar(slipAngle, 0);
    vx         = local_safe_scalar(vx, 0);

    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    if ~isfield(ctrlState, 'prevError')
        ctrlState.prevError = 0;
    end

    %% 2. 속도 가변 게인 스케줄링 (Gain Scheduling)
    % 힌트 기준속도 설정 (지정되지 않은 경우 기본값 15 m/s 적용)
    v_ref = 15;
    if isfield(CTRL, 'v_ref')
        v_ref = CTRL.v_ref;
    end

    % 종방향 속도 분모 제로 디바이드 방지 및 팩터 계산
    vx_safe = max(vx, 0.1);
    f_vx = min(vx_safe / v_ref, 2.0);

    % [AFS 게인 스케줄링]
    % 고속 주행 시 차량의 횡방향 민감도가 급격히 증가하므로 조향 게인을 낮추어 안정성을 확보합니다.
    % 저속(f_vx -> 0)일 때는 반응성을 높이고, 고속(f_vx -> 2)일 때는 기본 게인 수준을 유지하도록 설계
    gain_scale_afs = 1.5 - 0.25 * f_vx;
    Kp = CTRL.LAT.Kp * gain_scale_afs;
    Ki = CTRL.LAT.Ki * gain_scale_afs;
    Kd = CTRL.LAT.Kd * gain_scale_afs;

    %% 3. AFS (Active Front Steering) - Yaw Rate 추종 제어 (PID)
    % 오차 계산 (목표값 - 현재값)
    errorYawRate = yawRateRef - yawRate;

    % 비례항(P) 및 미분항(D) 계산
    P_term = Kp * errorYawRate;
    D_term = Kd * (errorYawRate - ctrlState.prevError) / dt;

    % 적분항(I) 업데이트 및 Anti-Windup (Clamping 방식)
    % 제어 입력이 포화되기 전 적분 오차 자체를 한계치로 제한하여 오버슛을 방지합니다.
    intMax = CTRL.LAT.intMax;
    ctrlState.intError = ctrlState.intError + errorYawRate * dt;
    ctrlState.intError = max(min(ctrlState.intError, intMax), -intMax);
    I_term = Ki * ctrlState.intError;

    % 제어 명령 조합 및 Saturation 처리
    steerCmd = P_term + I_term + D_term;
    maxSteer = LIM.MAX_STEER_ANGLE;
    deltaAdd.steerAngle = max(min(steerCmd, maxSteer), -maxSteer);

    %% 4. ESC (Electronic Stability Control) - Slip Angle 제한 (beta-Limiter)
    % 슬립각 임계값 설정 (LIM 구조체 내 변수가 없다면 MAX 값의 80%를 마진으로 적용)
    if isfield(LIM, 'SLIP_ANGLE_THRESHOLD')
        beta_th = LIM.SLIP_ANGLE_THRESHOLD;
    else
        beta_th = LIM.MAX_SLIP_ANGLE * 0.8;
    end

    % ESC 복원 모멘트 게인 설정 (구조체 유연성 확보)
    if isfield(CTRL, 'Kbeta')
        K_beta = CTRL.Kbeta;
    elseif isfield(CTRL, 'ESC') && isfield(CTRL.ESC, 'Kp')
        K_beta = CTRL.ESC.Kp;
    else
        K_beta = 20000; % Default 복원 요모멘트 게인 [Nm/rad]
    end

    % 차체 슬립각 제한 조건 판단
    abs_slip = abs(slipAngle);
    if abs_slip > beta_th
        % 힌트 공식 반영: M_z = -K_beta * sign(beta) * (|beta| - beta_th) * f(vx)
        % 차량 스핀을 억제하기 위해 슬립각 진행 방향과 정반대(오버스티어 제어)로 복원 모멘트 인가
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs_slip - beta_th) * f_vx;
    else
        deltaAdd.yawMoment = 0;
    end

    %% 5. 내부 상태 업데이트 (다음 스텝용)
    ctrlState.prevError = errorYawRate;
    ctrlState.lastSteerAngle = deltaAdd.steerAngle;
    ctrlState.lastYawMoment = deltaAdd.yawMoment;

    % Pass measured motion states downstream for straight-brake gating.
    deltaAdd.yawRateRef = yawRateRef;
    deltaAdd.measuredYawRate = yawRate;
    deltaAdd.measuredSlipAngle = slipAngle;

end

%% ------------------------------------------------------------------------
function x = local_safe_scalar(x, defaultValue)
    if ~isscalar(x) || ~isfinite(x)
        x = defaultValue;
    end
end
