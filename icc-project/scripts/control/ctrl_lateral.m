function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt, pathInfo)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 beta [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%       pathInfo   - optional struct(.hasPath, .lateralDev, .headingError)
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > beta_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 - 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리

    %% Robust defaults / input sanitizing
    if nargin < 8 || ~isfinite(dt) || dt <= 0
        dt = 0.01;
    end
    if nargin < 5 || ~isstruct(ctrlState)
        ctrlState = struct();
    end
    if nargin < 9 || ~isstruct(pathInfo)
        pathInfo = struct();
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
    steerAssistLimit = min(steerHardLimit, deg2rad(5.0));
    yawRateHardLimit = abs(local_get_nested(LIM, {'MAX_YAW_RATE'}, deg2rad(60)));
    ayHardLimit = abs(local_get_nested(LIM, {'MAX_AY'}, 9.81));
    slipHardLimit = abs(local_get_nested(LIM, {'MAX_SLIP_ANGLE'}, deg2rad(12)));

    vxAbs = abs(vx);
    vxEff = max(vxAbs, 0.5);
    speedBlend = local_sat((vxAbs - 0.5) / 2.5, 0, 1);
    brakeLikeSlip = local_sat(abs(slipAngle) / deg2rad(6), 0, 1);

    % Table-based gain scheduling: high-speed maneuvers keep enough P
    % authority for path response, while I action is nearly removed to
    % avoid phase-lag fishtailing in A3/A1/D1.
    speedGrid = [0, 5, 15, 25, 35];
    kpGrid = [1.05, 1.00, 0.96, 0.90, 0.88];
    kiGrid = [1.00, 0.85, 0.32, 0.10, 0.06];
    kpSched = local_interp1_clamped(speedGrid, kpGrid, vxAbs);
    kiSched = local_interp1_clamped(speedGrid, kiGrid, vxAbs) * (1.0 - 0.35 * brakeLikeSlip);

    yawRateRefLimit = min(yawRateHardLimit, ayHardLimit / vxEff);
    yawRateRefSafe = local_sat(yawRateRef, -yawRateRefLimit, yawRateRefLimit);
    yawRateSafe = local_sat(yawRate, -1.5 * yawRateHardLimit, 1.5 * yawRateHardLimit);
    yawErr = yawRateRefSafe - yawRateSafe;
    yawErrDot = (yawErr - ctrlState.prevError) / max(dt, 1e-4);

    %% AFS: PID yaw-rate tracking with gain scheduling + anti-windup
    kpEff = kp * kpSched;
    kiEff = ki * kiSched;
    % Disable raw finite-difference D action; unfiltered yawErrDot caused
    % chattering and unrealistically fast yaw-rate rise in step steering.
    kdEff = 0;

    intEffMax = min(0.05 * intMax, intMax * kiSched);
    intCandidate = local_sat(ctrlState.intError + yawErr * dt, -intEffMax, intEffMax);
    wheelbaseFF = 2.7;
    if vxAbs > 5.0
        steerFF = 1.05 * wheelbaseFF * yawRateRefSafe / max(vxAbs, 1.0);
    else
        steerFF = 0;
    end
    steerUnsat = speedBlend * (kpEff * yawErr + kiEff * intCandidate + kdEff * yawErrDot + steerFF);

    if abs(steerUnsat) <= steerAssistLimit || sign(steerUnsat) ~= sign(yawErr)
        ctrlState.intError = intCandidate;
        steerUnsat = speedBlend * (kpEff * yawErr + kiEff * ctrlState.intError + kdEff * yawErrDot + steerFF);
    end

    %% Steady/benign corner guard
    % In steady circular driving and path-following DLC the driver model
    % already carries the intended curvature. Keep AFS modest unless the
    % yaw error is large enough to be a stability problem.
    steadyYawGuard = (abs(yawRateRefSafe) > deg2rad(5)) && ...
                     (abs(yawErr) < 0.20 * max(abs(yawRateRefSafe), deg2rad(5)));
    if steadyYawGuard && abs(slipAngle) < deg2rad(2.5)
        steerUnsat = 0.50 * steerUnsat;
    end

    %% Path-error feedback for DLC / path-following scenarios
    % Yaw-rate tracking keeps heading dynamics stable, but A1/D1 also score
    % geometric path deviation. Use a small cross-track correction only when
    % the scenario provides a reference path.
    hasPath = local_get_nested(pathInfo, {'hasPath'}, false);
    lateralDev = local_get_nested(pathInfo, {'lateralDev'}, 0);
    headingErr = local_get_nested(pathInfo, {'headingError'}, 0);
    pathYawAssist = 0;
    if hasPath && isfinite(lateralDev) && isfinite(headingErr) && vxAbs > 5.0
        pathBlend = local_sat((vxAbs - 5.0) / 10.0, 0, 1);
        latErrCtrl = local_sat(lateralDev, -2.0, 2.0);
        headingCtrl = local_sat(headingErr, -deg2rad(12), deg2rad(12));
        pathSteer = pathBlend * (0.060 * latErrCtrl + 0.10 * headingCtrl);
        pathSteer = local_sat(pathSteer, -deg2rad(3.0), deg2rad(3.0));

        % If the body is already slipping, protect A4/A7-like stability by
        % fading the geometric correction rather than adding more tire slip.
        slipFade = 1.0 - local_sat((abs(slipAngle) - deg2rad(2.0)) / deg2rad(3.0), 0, 0.75);
        steerUnsat = steerUnsat + slipFade * pathSteer;
        pathYawAssist = slipFade * pathBlend * local_sat(0.22 * latErrCtrl + 0.18 * headingCtrl / deg2rad(8), -1, 1);
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

    mzTrack = speedBlend * 0.35 * yawMomentLimit * local_sat(yawNorm, -1, 1);
    mzSlip  = speedBlend * yawMomentLimit * local_sat(betaNorm, -1, 1);
    mzPath  = speedBlend * 0.12 * yawMomentLimit * pathYawAssist;

    if betaExcess > 0
        yawMomentCmd = mzTrack + mzSlip + 0.30 * mzPath;
    else
        if abs(yawNorm) > 0.25
            yawMomentCmd = 0.85 * mzTrack + mzPath;
        else
            yawMomentCmd = mzPath;
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
    deltaAdd.lateralDev = local_get_nested(pathInfo, {'lateralDev'}, 0);

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

function y = local_interp1_clamped(xGrid, yGrid, x)
    if x <= xGrid(1)
        y = yGrid(1);
        return;
    end
    if x >= xGrid(end)
        y = yGrid(end);
        return;
    end
    idx = find(xGrid <= x, 1, 'last');
    idx = min(idx, numel(xGrid) - 1);
    dx = xGrid(idx+1) - xGrid(idx);
    if dx <= 0
        y = yGrid(idx);
        return;
    end
    a = (x - xGrid(idx)) / dx;
    y = (1 - a) * yGrid(idx) + a * yGrid(idx+1);
end
