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

    %% Robust defaults / input sanitizing
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

    if ~isfield(ctrlState, 'intError') || ~isscalar(ctrlState.intError) || ~isfinite(ctrlState.intError)
        ctrlState.intError = 0;
    end
    if ~isfield(ctrlState, 'prevError') || ~isscalar(ctrlState.prevError) || ~isfinite(ctrlState.prevError)
        ctrlState.prevError = 0;
    end

    %% Controller parameters and guards
    kp = local_get_nested(CTRL, {'LAT','Kp'}, 1.0);
    ki = local_get_nested(CTRL, {'LAT','Ki'}, 0.1);
    kd = local_get_nested(CTRL, {'LAT','Kd'}, 0.05);
    intMax = abs(local_get_nested(CTRL, {'LAT','intMax'}, 5.0));

    steerHardLimit = abs(local_get_nested(LIM, {'MAX_STEER_ANGLE'}, deg2rad(30)));
    steerAssistLimit = min(steerHardLimit, deg2rad(4.5));
    yawRateHardLimit = abs(local_get_nested(LIM, {'MAX_YAW_RATE'}, deg2rad(60)));
    ayHardLimit = abs(local_get_nested(LIM, {'MAX_AY'}, 9.81));
    slipHardLimit = abs(local_get_nested(LIM, {'MAX_SLIP_ANGLE'}, deg2rad(12)));

    vxAbs = abs(vx);
    vxEff = max(vxAbs, 0.5);
    speedBlend = local_sat((vxAbs - 0.5) / 2.5, 0, 1);       % fade in above ~3 m/s
    speedSched = 0.7 + 0.7 * local_sat((vxAbs - 3.0) / 17.0, 0, 1);

    yawRateRefLimit = min(yawRateHardLimit, ayHardLimit / vxEff);
    yawRateRefSafe = local_sat(yawRateRef, -yawRateRefLimit, yawRateRefLimit);
    yawRateSafe = local_sat(yawRate, -1.5 * yawRateHardLimit, 1.5 * yawRateHardLimit);
    yawErr = yawRateRefSafe - yawRateSafe;
    yawErrDot = (yawErr - ctrlState.prevError) / max(dt, 1e-4);

    %% AFS: PID yaw-rate tracking with gain scheduling + anti-windup
    kpEff = kp * speedSched;
    kiEff = ki * (0.5 + 0.5 * speedSched);
    kdEff = kd * (0.4 + 0.6 * speedSched);

    intCandidate = local_sat(ctrlState.intError + yawErr * dt, -intMax, intMax);
    steerUnsat = speedBlend * (kpEff * yawErr + kiEff * intCandidate + kdEff * yawErrDot);

    % If the steering assist saturates in the same direction as the error,
    % freeze the integrator to avoid windup. Otherwise keep integrating.
    if abs(steerUnsat) <= steerAssistLimit || sign(steerUnsat) ~= sign(yawErr)
        ctrlState.intError = intCandidate;
        steerUnsat = speedBlend * (kpEff * yawErr + kiEff * ctrlState.intError + kdEff * yawErrDot);
    end

    %% Steady/benign corner guard
    % In steady circular driving and path-following DLC the driver model
    % already carries the intended curvature. Keep AFS modest unless the
    % yaw error is large enough to be a stability problem.
    steadyYawGuard = (abs(yawRateRefSafe) > deg2rad(3)) && ...
                     (abs(yawErr) < 0.30 * max(abs(yawRateRefSafe), deg2rad(3)));
    if steadyYawGuard && abs(slipAngle) < deg2rad(3.5)
        steerUnsat = 0.18 * steerUnsat;
    end

    deltaAdd.steerAngle = local_sat(steerUnsat, -steerAssistLimit, steerAssistLimit);

    %% ESC: slip-angle limiter + light yaw-rate support
    betaThreshold = min(deg2rad(3.0), 0.75 * slipHardLimit);
    betaExcess = max(abs(slipAngle) - betaThreshold, 0);
    betaError = sign(slipAngle) * betaExcess;

    % Positive yaw moment must support the plant sign convention:
    % larger left-side brake torque -> positive (CCW) yaw moment.
    yawMomentLimit = 1400 + 1000 * local_sat((vxAbs - 8.0) / 20.0, 0, 1);
    betaNorm = betaError / max(slipHardLimit - betaThreshold, deg2rad(1));
    yawNorm  = yawErr / max(yawRateRefLimit, deg2rad(5));
    yawRateNorm = yawRateSafe / max(yawRateRefLimit, deg2rad(5));

    mzTrack = speedBlend * 0.35 * yawMomentLimit * local_sat(yawNorm, -1, 1);
    mzSlip  = speedBlend * yawMomentLimit * local_sat(betaNorm, -1, 1);

    if betaExcess > 0
        yawMomentCmd = mzTrack + mzSlip;
    else
        % Keep ESC dormant in benign conditions. This preserves path and
        % steady-state cornering KPIs; ESC wakes only for large yaw errors.
        dampReady = abs(yawRateSafe) > 0.80 * max(abs(yawRateRefSafe), deg2rad(2)) && ...
                    sign(yawRateSafe) == sign(yawRateRefSafe);
        if dampReady
            yawDamp = -0.32 * speedBlend * yawMomentLimit * local_sat(yawRateNorm, -1, 1);
            yawMomentCmd = yawDamp;
        elseif abs(yawNorm) > 0.55
            yawMomentCmd = 0.08 * mzTrack;
        else
            yawMomentCmd = 0;
        end
    end

    deltaAdd.yawMoment = local_sat(yawMomentCmd, -yawMomentLimit, yawMomentLimit);

    %% Housekeeping
    ctrlState.prevError = yawErr;
    ctrlState.lastSteerAngle = deltaAdd.steerAngle;
    ctrlState.lastYawMoment = deltaAdd.yawMoment;

    % Pass measured motion states downstream for straight-brake gating.
    deltaAdd.yawRateRef = yawRateRef;
    deltaAdd.measuredYawRate = yawRate;
    deltaAdd.measuredSlipAngle = slipAngle;

end

%% ------------------------------------------------------------------------
function value = local_get_nested(s, fields, defaultValue)
    value = defaultValue;
    if ~isstruct(s)
        return;
    end
    cur = s;
    for i = 1:numel(fields)
        if ~isstruct(cur) || ~isfield(cur, fields{i})
            return;
        end
        cur = cur.(fields{i});
    end
    if isscalar(cur) && isfinite(cur)
        value = cur;
    end
end

function x = local_safe_scalar(x, defaultValue)
    if ~isscalar(x) || ~isfinite(x)
        x = defaultValue;
    end
end

function y = local_sat(x, lower, upper)
    y = min(max(x, lower), upper);
end
