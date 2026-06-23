function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC — per-wheel 가변 감쇠 (continuous skyhook + groundhook 하이브리드)
%
%   suspState.zs_dot / zu_dot (4×1) 사용. (suspState 비면 nominal 반환)
%
%   원리:
%     - skyhook : sprung mass 절대속도(zs_dot) 억제 → body bounce 저감 (승차감)
%     - groundhook : unsprung mass 절대속도(zu_dot) 억제 → wheel hop 저감 (접지)
%     - relative velocity (zs_dot - zu_dot) 부호로 semi-active 가능 영역 판정

    if ~isfield(ctrlState,'init'); ctrlState.init = true; end

    c_nom = 1500;
    cMin  = 500;  cMax = 5000; skyGain = 2500;
    if isfield(CTRL,'VER')
        if isfield(CTRL.VER,'cMin');    cMin    = CTRL.VER.cMin;    end
        if isfield(CTRL.VER,'cMax');    cMax    = CTRL.VER.cMax;    end
        if isfield(CTRL.VER,'skyGain'); skyGain = CTRL.VER.skyGain; end
    end

    % suspState 미제공(저DOF plant) → passive nominal
    if isempty(fieldnames(suspState)) || ~isfield(suspState,'zs_dot')
        dampingCmd = c_nom * ones(4,1);
        return;
    end

    zs_dot = suspState.zs_dot(:);
    if isfield(suspState,'zu_dot'); zu_dot = suspState.zu_dot(:); else; zu_dot = zeros(4,1); end
    vrel = zs_dot - zu_dot;   % suspension 압축/신장 속도

    alpha = 0.7;
    dampingCmd = zeros(4,1);
    % 압축(vrel<0, 하중 증가측=바깥바퀴)에서 댐핑 강화 → 롤/LTR 억제
    for i = 1:4
        if abs(vrel(i)) > 1e-4
            c_sky = skyGain *  zs_dot(i) / vrel(i);
            c_grd = skyGain * (-zu_dot(i)) / vrel(i);
            c_i = alpha*c_sky + (1-alpha)*c_grd;
        else
            c_i = c_nom;
        end
        % 서스펜션 압축 중이면(바깥쪽 하중 증가) 댐핑 가산
        if vrel(i) < 0
            c_i = c_i + 1500;
        end
        dampingCmd(i) = max(cMin, min(cMax, c_i));
    end
end
