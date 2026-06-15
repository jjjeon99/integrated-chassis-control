function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

    %% Robust defaults / input sanitizing
    if nargin < 4 || ~isfinite(dt) || dt <= 0
        dt = 0.01;
    end
    if nargin < 2 || ~isstruct(ctrlState)
        ctrlState = struct();
    end
    if nargin < 1 || ~isstruct(suspState)
        suspState = struct();
    end

    cMin = abs(local_get_nested(CTRL, {'VER','cMin'}, 500));
    cMax = abs(local_get_nested(CTRL, {'VER','cMax'}, 5000));
    cMax = max(cMax, cMin + 1);
    skyGain = abs(local_get_nested(CTRL, {'VER','skyGain'}, 2500));

    cNom = 0.5 * (cMin + cMax);
    zsDot = local_get_vec4(suspState, 'zs_dot', 0);
    zuDot = local_get_vec4(suspState, 'zu_dot', 0);
    zs    = local_get_vec4(suspState, 'zs', 0);
    zu    = local_get_vec4(suspState, 'zu', 0);

    relVel = zsDot - zuDot;
    relDisp = zs - zu;

    %% Modal indicators for heave/pitch/roll sensitivity
    heaveVel = mean(zsDot);
    pitchVel = 0.5 * ((zsDot(1) + zsDot(2)) - (zsDot(3) + zsDot(4)));
    rollVel  = 0.5 * ((zsDot(1) + zsDot(3)) - (zsDot(2) + zsDot(4)));
    bodyMotionBlend = local_sat( ...
        abs(heaveVel) / 0.25 + abs(pitchVel) / 0.35 + abs(rollVel) / 0.35, ...
        0, 1.5);

    dampingCmd = cNom * ones(4, 1);
    for i = 1:4
        relVelEff = sign(relVel(i)) * max(abs(relVel(i)), 0.05);

        % Continuous skyhook target: F = c * (zs_dot - zu_dot) ~= skyGain * zs_dot
        cSky = skyGain * abs(zsDot(i)) / max(abs(relVelEff), 0.05);
        cSky = cSky * (1.0 + 0.20 * bodyMotionBlend);
        cSky = local_sat(cSky, cMin, cMax);

        % Clipped skyhook: only apply high damping when it extracts energy
        % from the sprung mass. Otherwise fall back to minimum damping.
        if zsDot(i) * relVel(i) > 0
            cCmd = cSky;
        else
            cCmd = cMin;
        end

        % Mild groundhook support for wheel-hop suppression.
        if zuDot(i) * relVel(i) < 0
            cGround = cMin + 0.25 * (cMax - cMin) * local_sat(abs(zuDot(i)) / 0.30, 0, 1);
            cCmd = max(cCmd, cGround);
        end

        % Extra damping under front dive / rear rebound to support braking stability.
        if i <= 2
            pitchDive = local_sat(-pitchVel / 0.25, 0, 1);
            cCmd = cCmd + 0.15 * (cMax - cMin) * pitchDive;
        else
            rearSupport = local_sat(abs(relDisp(i)) / 0.03, 0, 1);
            cCmd = cCmd + 0.05 * (cMax - cMin) * rearSupport;
        end

        % Extra roll-rate damping during fast lane-change transients. This
        % targets A1/D1 LTR peaks without adding steady yaw/steer action.
        rollSupport = local_sat(abs(rollVel) / 0.25, 0, 1);
        cCmd = cCmd + 0.08 * (cMax - cMin) * rollSupport;

        dampingCmd(i) = local_sat(cCmd, cMin, cMax);
    end

    % Light first-order smoothing to avoid harsh coefficient steps.
    if isfield(ctrlState, 'prevDamping')
        prevDamping = local_safe_vec4(ctrlState.prevDamping, cNom);
        alpha = local_sat(dt / 0.02, 0, 1);  % ~20 ms time constant
        dampingCmd = prevDamping + alpha * (dampingCmd - prevDamping);
        dampingCmd = local_sat(dampingCmd, cMin, cMax);
    end

    ctrlState.prevDamping = dampingCmd;

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

function vec = local_get_vec4(s, fieldName, defaultValue)
    if ~isstruct(s) || ~isfield(s, fieldName)
        vec = defaultValue * ones(4, 1);
        return;
    end
    vec = local_safe_vec4(s.(fieldName), defaultValue);
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
