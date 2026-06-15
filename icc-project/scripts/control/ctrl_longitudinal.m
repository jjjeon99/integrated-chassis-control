function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       forceCmd.brakeAssistRatio - 외부 직진 제동 시 추가 제동 요청 비율
%       forceCmd.brakeAssistWheelRatio - wheel별 ABS 보정 비율
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% Robust defaults / input sanitizing
    if nargin < 7 || ~isfinite(dt) || dt <= 0
        dt = 0.01;
    end
    if nargin < 4 || ~isstruct(ctrlState)
        ctrlState = struct();
    end

    vxRef = local_safe_scalar(vxRef, 0);
    vx    = local_safe_scalar(vx, 0);
    ax    = local_safe_scalar(ax, 0);

    if ~isfield(ctrlState, 'intError') || ~isscalar(ctrlState.intError) || ~isfinite(ctrlState.intError)
        ctrlState.intError = 0;
    end
    if ~isfield(ctrlState, 'prevForce') || ~isscalar(ctrlState.prevForce) || ~isfinite(ctrlState.prevForce)
        ctrlState.prevForce = 0;
    end
    if ~isfield(ctrlState, 'prevBrakeAssistRatio')
        ctrlState.prevBrakeAssistRatio = zeros(4, 1);
    end
    ctrlState.prevBrakeAssistRatio = local_safe_vec4(ctrlState.prevBrakeAssistRatio, 0);
    if ~isfield(ctrlState, 'wheelSlip')
        ctrlState.wheelSlip = zeros(4, 1);
    end
    ctrlState.wheelSlip = local_safe_vec4(ctrlState.wheelSlip, 0);

    %% Controller parameters and limits
    kp = local_get_nested(CTRL, {'LON','Kp'}, 0.5);
    ki = local_get_nested(CTRL, {'LON','Ki'}, 0.05);
    intMax = abs(local_get_nested(CTRL, {'LON','intMax'}, 2000));

    maxAx = abs(local_get_nested(LIM, {'MAX_AX'}, 10.0));
    maxJerk = abs(local_get_nested(LIM, {'MAX_JERK'}, 50.0));
    maxBrakeTrq = abs(local_get_nested(LIM, {'MAX_BRAKE_TRQ'}, 3000));

    % VEH is not provided here, so use a conservative nominal vehicle mass
    % and wheel radius for force scaling.
    mEst = 1500;    % [kg]
    rwNom = 0.31;   % [m]
    maxDriveForce = mEst * maxAx;
    maxBrakeForce = min(4 * maxBrakeTrq / rwNom, 1.2 * mEst * maxAx);

    %% PI speed tracking
    vxErr = vxRef - vx;
    intCandidate = local_sat(ctrlState.intError + vxErr * dt, -intMax, intMax);
    fxUnsat = mEst * (kp * vxErr + ki * intCandidate);

    % During an ongoing braking event, do not push against the external
    % deceleration with a positive drive request.
    externalBrakeActive = (ax < -0.5) && (vx > 1.0);
    if externalBrakeActive
        fxUnsat = min(fxUnsat, 0);
    end

    fxLimited = local_sat(fxUnsat, -maxBrakeForce, maxDriveForce);
    limitForUnsat = maxDriveForce;
    if fxUnsat < 0
        limitForUnsat = maxBrakeForce;
    end
    allowIntegrator = (~externalBrakeActive) || (vxErr < 0);
    if allowIntegrator && (abs(fxUnsat) <= limitForUnsat || sign(fxUnsat) ~= sign(vxErr))
        ctrlState.intError = intCandidate;
        fxLimited = local_sat(mEst * (kp * vxErr + ki * ctrlState.intError), -maxBrakeForce, maxDriveForce);
        if externalBrakeActive
            fxLimited = min(fxLimited, 0);
        end
    end

    %% ABS-style brake modulation using cached wheel slip
    % Runner can inject previous-step slip ratios via ctrlState.wheelSlip.
    % If slip is unavailable, this block gracefully falls back to PI only.
    slipAbs = abs(ctrlState.wheelSlip(:));
    absTarget = 0.12;
    absSlipLow = 0.08;
    absSlipHigh = 0.15;

    brakeForceReq = max(0, -fxLimited);
    absScale = 1.0;
    absActive = (brakeForceReq > 0) && (ax < -0.2 || max(slipAbs) > absTarget);
    if absActive
        peakSlip = max(slipAbs);
        meanSlip = mean(slipAbs);
        if peakSlip > absSlipHigh
            absScale = local_sat(1.0 - 4.0 * (peakSlip - absSlipHigh), 0.2, 1.0);
        elseif meanSlip < absSlipLow && vx > 1.0
            absScale = local_sat(1.0 + 1.5 * (absSlipLow - meanSlip), 1.0, 1.1);
        end
    end

    if brakeForceReq > 0
        fxLimited = -brakeForceReq * absScale;
    end

    %% Jerk limiting on the total longitudinal force request
    maxForceStep = maxJerk * mEst * dt;
    fxRateLimited = local_sat(fxLimited, ...
        ctrlState.prevForce - maxForceStep, ...
        ctrlState.prevForce + maxForceStep);

    % Additional low-speed protection: fade extra controller braking near stop.
    if vx < 1.0 && fxRateLimited < 0
        fxRateLimited = fxRateLimited * local_sat(vx / 1.0, 0.0, 1.0);
    end

    forceCmd.Fx_total = local_sat(fxRateLimited, -maxBrakeForce, maxDriveForce);
    forceCmd.brakeRatio = local_sat(max(0, -forceCmd.Fx_total) / max(maxBrakeForce, 1), 0, 1);
    forceCmd.brakeAssistRatio = 0;
    forceCmd.brakeAssistWheelRatio = zeros(4, 1);

    % B1 straight braking uses a stronger external brake step than A7/D1.
    % Keep each wheel near the ABS slip target: add torque when slip is low,
    % but request brake relief when an individual wheel is over-slip.
    hardBrakeActive = externalBrakeActive && ax < -3.8 && vx > 3.0;
    brakeSlip = max(0, -ctrlState.wheelSlip(:));
    meanBrakeSlip = mean(brakeSlip);
    peakBrakeSlip = max(brakeSlip);
    if hardBrakeActive
        slipTarget = 0.12;
        slipErr = slipTarget - brakeSlip;
        wheelAssistTarget = zeros(4, 1);
        addMask = slipErr >= 0;
        wheelAssistTarget(addMask) = 0.9 * slipErr(addMask);
        wheelAssistTarget(~addMask) = 7.5 * slipErr(~addMask);
        wheelAssistTarget = local_sat(wheelAssistTarget, -0.70, 0.12);

        % If all cached slips are still unavailable/zero at brake onset,
        % apply a short conservative push so the controller visibly engages.
        if peakBrakeSlip < 1e-4
            wheelAssistTarget = 0.08 * ones(4, 1);
        end

    else
        wheelAssistTarget = zeros(4, 1);
    end
    assistStep = 10.0 * dt;
    forceCmd.brakeAssistWheelRatio = local_sat(wheelAssistTarget, ...
        ctrlState.prevBrakeAssistRatio - assistStep, ...
        ctrlState.prevBrakeAssistRatio + assistStep);
    forceCmd.brakeAssistWheelRatio = local_sat(forceCmd.brakeAssistWheelRatio, -0.70, 0.12);
    forceCmd.brakeAssistRatio = local_sat(mean(forceCmd.brakeAssistWheelRatio), -0.70, 0.12);

    ctrlState.prevForce = forceCmd.Fx_total;
    ctrlState.absActive = absActive;
    ctrlState.absScale = absScale;
    ctrlState.brakeAssistRatio = forceCmd.brakeAssistRatio;
    ctrlState.prevBrakeAssistRatio = forceCmd.brakeAssistWheelRatio;
    ctrlState.hardBrakeActive = hardBrakeActive;
    ctrlState.meanBrakeSlip = meanBrakeSlip;
    ctrlState.peakBrakeSlip = peakBrakeSlip;

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

function vec = local_safe_vec4(vec, defaultValue)
    if ~isnumeric(vec) || isempty(vec)
        vec = defaultValue * ones(4, 1);
    end
    vec = vec(:);
    if numel(vec) < 4
        vec(end+1:4,1) = defaultValue;
    elseif numel(vec) > 4
        vec = vec(1:4);
    end
    bad = ~isfinite(vec);
    vec(bad) = defaultValue;
end

function y = local_sat(x, lower, upper)
    y = min(max(x, lower), upper);
end
