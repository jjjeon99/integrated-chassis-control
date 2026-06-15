function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       lonCmd.brakeAssistRatio - 외부 직진 제동 보조 비율
%       lonCmd.brakeAssistWheelRatio - wheel별 ABS 제동 보정 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% Robust defaults / input sanitizing
    if nargin < 1 || ~isstruct(latCmd); latCmd = struct(); end
    if nargin < 2 || ~isstruct(lonCmd); lonCmd = struct(); end
    if nargin < 5 || ~isstruct(VEH); VEH = struct(); end
    if nargin < 6 || ~isstruct(CTRL); CTRL = struct(); end
    if nargin < 7 || ~isstruct(LIM); LIM = struct(); end

    vx = local_safe_scalar(vx, 0);

    steerReq = local_get_nested(latCmd, {'steerAngle'}, 0);
    yawMomentReq = local_get_nested(latCmd, {'yawMoment'}, 0);
    yawRateRef = local_get_nested(latCmd, {'yawRateRef'}, 0);
    measuredYawRate = local_get_nested(latCmd, {'measuredYawRate'}, 0);
    measuredSlipAngle = local_get_nested(latCmd, {'measuredSlipAngle'}, 0);
    fxTotalReq = local_get_nested(lonCmd, {'Fx_total'}, 0);
    brakeRatio = local_sat(local_get_nested(lonCmd, {'brakeRatio'}, 0), 0, 1);
    brakeAssistRatio = local_sat(local_get_nested(lonCmd, {'brakeAssistRatio'}, 0), 0, 1);
    brakeAssistWheelRatio = local_get_vec4(lonCmd, 'brakeAssistWheelRatio', brakeAssistRatio);
    brakeAssistWheelRatio = local_sat(brakeAssistWheelRatio, -1, 1);

    rw = abs(local_get_nested(VEH, {'rw'}, 0.31));
    trackF = max(abs(local_get_nested(VEH, {'track_f'}, 1.55)), 0.5);
    trackR = max(abs(local_get_nested(VEH, {'track_r'}, 1.55)), 0.5);
    maxBrakeTrq = abs(local_get_nested(LIM, {'MAX_BRAKE_TRQ'}, 3000));
    maxSteerAngle = abs(local_get_nested(LIM, {'MAX_STEER_ANGLE'}, deg2rad(30)));
    cMin = abs(local_get_nested(CTRL, {'VER','cMin'}, 500));
    cMax = abs(local_get_nested(CTRL, {'VER','cMax'}, 5000));
    cMax = max(cMax, cMin + 1);

    %% AFS pass-through with saturation
    actuatorCmd.steerAngle = local_sat(steerReq, -maxSteerAngle, maxSteerAngle);

    %% Vertical command pass-through with safety clipping
    actuatorCmd.dampingCoeff = local_safe_vec4(verCmd, 0.5 * (cMin + cMax));
    actuatorCmd.dampingCoeff = local_sat(actuatorCmd.dampingCoeff, cMin, cMax);

    %% Base brake allocation from longitudinal demand
    % Fx_total < 0 means braking. Convert total brake force to total wheel
    % torque and split 60:40 front/rear.
    totalBrakeForce = max(0, -fxTotalReq);
    totalBrakeTorque = totalBrakeForce * rw;

    % If only brakeRatio is populated, fall back to a moderate torque demand.
    if totalBrakeTorque < 1 && brakeRatio > 0
        totalBrakeTorque = brakeRatio * 2.0 * maxBrakeTrq;
    end

    baseBrake = totalBrakeTorque * [0.30; 0.30; 0.20; 0.20];

    % Longitudinal brake assist: only boost when the vehicle is braking
    % nearly straight, so brake-in-turn ESC/AFS behavior is left unchanged.
    isStraightBrake = abs(yawRateRef) < 0.03 && ...
                      abs(measuredYawRate) < 0.05 && ...
                      abs(measuredSlipAngle) < 0.05 && ...
                      abs(steerReq) < deg2rad(1.5) && ...
                      abs(yawMomentReq) < 100 && ...
                      (brakeRatio > 0.5 || max(abs(brakeAssistWheelRatio)) > 0);
    if isStraightBrake
        brakeBoostGain = 1.07;
        baseBrake = baseBrake * brakeBoostGain;

        if max(abs(brakeAssistWheelRatio)) > 0
            assistBrake = zeros(4, 1);
            addMask = brakeAssistWheelRatio > 0;
            relMask = brakeAssistWheelRatio < 0;
            assistBrake(addMask) = 2.0 * maxBrakeTrq * 0.22 * brakeAssistWheelRatio(addMask);
            assistBrake(relMask) = 2.0 * maxBrakeTrq * 0.34 * brakeAssistWheelRatio(relMask);
            baseBrake = baseBrake + assistBrake;
        end
    end

    if isStraightBrake
        baseBrake = local_sat(baseBrake, -0.8 * maxBrakeTrq, maxBrakeTrq);
    else
        baseBrake = local_sat(baseBrake, 0, maxBrakeTrq);
    end

    %% ESC yaw-moment allocation via differential braking
    % Plant sign convention:
    %   positive yaw moment (CCW) <=> left-side brake torque > right-side.
    yawBlend = local_sat((abs(vx) - 1.0) / 4.0, 0, 1);
    ratioFront = 0.60;
    yawMomentReq = yawBlend * yawMomentReq;
    if isStraightBrake
        yawMomentReq = 0;
    end

    diffFront = -2 * (ratioFront * yawMomentReq) * rw / trackF;            % T_R - T_L
    diffRear  = -2 * ((1 - ratioFront) * yawMomentReq) * rw / trackR;      % T_R - T_L

    frontPair = local_apply_yaw_pair(baseBrake(1), baseBrake(2), diffFront, maxBrakeTrq);
    rearPair  = local_apply_yaw_pair(baseBrake(3), baseBrake(4), diffRear,  maxBrakeTrq);

    actuatorCmd.brakeTorque = [frontPair; rearPair];
    if isStraightBrake
        actuatorCmd.brakeTorque = local_sat(actuatorCmd.brakeTorque, -0.8 * maxBrakeTrq, maxBrakeTrq);
    else
        actuatorCmd.brakeTorque = local_sat(actuatorCmd.brakeTorque, 0, maxBrakeTrq);
    end

end

%% ------------------------------------------------------------------------
function pair = local_apply_yaw_pair(leftBase, rightBase, diffReq, maxBrakeTrq)
% Allocate yaw-induced differential brake on one axle.
% diffReq is defined as T_right - T_left.
    pair = [leftBase; rightBase];
    if ~isfinite(diffReq) || abs(diffReq) < 1e-9
        return;
    end

    if diffReq >= 0
        diffMag = diffReq;

        % Preserve total brake demand when possible: first release the
        % opposite wheel, then add torque on the target wheel if needed.
        leftRelease = min(pair(1), 0.5 * diffMag);
        pair(1) = pair(1) - leftRelease;

        remaining = diffMag - leftRelease;
        rightAdd = min(maxBrakeTrq - pair(2), remaining);
        pair(2) = pair(2) + rightAdd;
    else
        diffMag = -diffReq;

        rightRelease = min(pair(2), 0.5 * diffMag);
        pair(2) = pair(2) - rightRelease;

        remaining = diffMag - rightRelease;
        leftAdd = min(maxBrakeTrq - pair(1), remaining);
        pair(1) = pair(1) + leftAdd;
    end
end

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

function vec = local_get_vec4(s, fieldName, defaultValue)
    if ~isstruct(s) || ~isfield(s, fieldName)
        vec = defaultValue * ones(4, 1);
        return;
    end
    vec = local_safe_vec4(s.(fieldName), defaultValue);
end

function y = local_sat(x, lower, upper)
    y = min(max(x, lower), upper);
end
