function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
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
    brakeAssistRatio = local_sat(local_get_nested(lonCmd, {'brakeAssistRatio'}, 0), -4.0, 1);
    brakeAssistWheelRatio = local_get_vec4(lonCmd, 'brakeAssistWheelRatio', brakeAssistRatio);
    brakeAssistWheelRatio = local_sat(brakeAssistWheelRatio, -4.0, 1);

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
    totalBrakeForce = max(0, -fxTotalReq);
    totalBrakeTorque = totalBrakeForce * rw;

    if totalBrakeTorque < 1 && brakeRatio > 0
        totalBrakeTorque = brakeRatio * 2.0 * maxBrakeTrq;
    end

    % 전후 60:40 기본 분배
    baseBrake = totalBrakeTorque * [0.30; 0.30; 0.20; 0.20];

    %% [수정] 순수 직진 강한 제동 시의 부스트 조건 분리
    isStraightBrake = abs(yawRateRef) < 0.03 && ...
                      abs(measuredYawRate) < 0.05 && ...
                      abs(measuredSlipAngle) < 0.05 && ...
                      abs(steerReq) < deg2rad(1.5) && ...
                      abs(yawMomentReq) < 100 && ...
                      (brakeRatio > 0.5 || max(abs(brakeAssistWheelRatio)) > 0);

    if isStraightBrake
        brakeBoostGain = 1.08;
        baseBrake = baseBrake * brakeBoostGain;
    end

    %% [수정] ABS 휠 보정 로직 분리 (직진/선회 불문 항상 탈출구 마련)
    % 슬립 발생 시 상위 제어기의 토크 저감/증가 명령이 항상 반영되도록 Block 외부로 배치
    if max(abs(brakeAssistWheelRatio)) > 0
        assistBrake = 2.0 * maxBrakeTrq * 0.25 * brakeAssistWheelRatio;
        baseBrake = baseBrake + assistBrake;
    end

    % Controller output is added to scenario brake in the runner. During
    % straight-brake ABS, negative torque is therefore a valid relief request
    % that reduces the externally commanded master-cylinder torque.
    if isStraightBrake
        baseBrake = local_sat(baseBrake, -0.85 * maxBrakeTrq, maxBrakeTrq);
    else
        baseBrake = local_sat(baseBrake, 0, maxBrakeTrq);
    end

    %% ESC yaw-moment allocation via differential braking
    yawBlend = local_sat((abs(vx) - 1.0) / 4.0, 0, 1);
    % 직진 제동 보조 상태가 아닐 때만 요모멘트 분배 활성화
    if isStraightBrake
        localYawMomentReq = 0;
    else
        localYawMomentReq = yawBlend * yawMomentReq;
    end

    yawBrakeDelta = local_wls_yaw_allocation(localYawMomentReq, baseBrake, rw, trackF, trackR, maxBrakeTrq);
    actuatorCmd.brakeTorque = baseBrake + yawBrakeDelta;
    if isStraightBrake
        actuatorCmd.brakeTorque = local_sat(actuatorCmd.brakeTorque, -0.85 * maxBrakeTrq, maxBrakeTrq);
    else
        actuatorCmd.brakeTorque = local_sat(actuatorCmd.brakeTorque, 0, maxBrakeTrq);
    end

end

%% ------------------------------------------------------------------------
function deltaBrake = local_wls_yaw_allocation(yawMomentReq, baseBrake, rw, trackF, trackR, maxBrakeTrq)
    deltaBrake = zeros(4, 1);
    if ~isfinite(yawMomentReq) || abs(yawMomentReq) < 1e-9
        return;
    end

    % A*dT approximates generated yaw moment. Positive yaw moment follows
    % the existing sign convention: more left-side brake torque yields
    % positive yaw.
    A = [trackF/(2*rw), -trackF/(2*rw), trackR/(2*rw), -trackR/(2*rw)];

    % Weighted least-squares effort: prefer front axle for ESC authority but
    % keep rear participation available when front brakes are saturated.
    headroom = max(maxBrakeTrq - baseBrake(:), 1);
    reliefRoom = max(baseBrake(:), 1);
    weight = [1.0; 1.0; 1.35; 1.35] .* (1.0 + 0.25 ./ max(headroom / maxBrakeTrq, 0.05));
    invW = diag(1 ./ max(weight, 1e-3));

    denom = A * invW * A';
    if denom < 1e-9 || ~isfinite(denom)
        return;
    end

    deltaBrake = invW * A' * (yawMomentReq / denom);

    % Keep the correction feasible before the final global saturation so the
    % allocator does not silently lose all yaw authority at one wheel.
    lower = -0.65 * reliefRoom;
    upper = 0.65 * headroom;
    deltaBrake = local_sat(deltaBrake, lower, upper);
end

function pair = local_apply_yaw_pair(leftBase, rightBase, diffReq, maxBrakeTrq)
    pair = [leftBase; rightBase];
    if ~isfinite(diffReq) || abs(diffReq) < 1e-9
        return;
    end

    if diffReq >= 0
        diffMag = diffReq;
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
    if ~isstruct(s); return; end
    cur = s;
    for i = 1:numel(fields)
        if ~isstruct(cur) || ~isfield(cur, fields{i}); return; end
        cur = cur.(fields{i});
    end
    if isscalar(cur) && isfinite(cur)
        value = cur;
    end
end

function x = local_safe_scalar(x, defaultValue)
    if ~isscalar(x) || ~isfinite(x); x = defaultValue; end
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
