function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
    if ~isfield(ctrlState,'intError');  ctrlState.intError = 0; end
    if ~isfield(ctrlState,'prevForce'); ctrlState.prevForce = 0; end
    if ~isfield(ctrlState,'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end
    m = 1500;

    err = vxRef - vx;
    ctrlState.intError = ctrlState.intError + err * dt;
    ctrlState.intError = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intError));
    Fx_track = CTRL.LON.Kp * m * err + CTRL.LON.Ki * m * ctrlState.intError;

    Fmax = m * LIM.MAX_AX;
    % ★ 제동饱和 1.4배 (적당히)
    if Fx_track < 0
        Fx_total = max(-Fmax * 1.4, min(0, Fx_track));
    else
        Fx_total = min(Fmax, max(0, Fx_track));
    end

    dF_max = m * LIM.MAX_JERK * dt;
    dF_max_brake = 2.0 * dF_max;   % ★ jerk 2.0배 (원래 1.8 → 2.0, 2.5에서 낮춤)
    dF = Fx_total - ctrlState.prevForce;
    if Fx_total < ctrlState.prevForce
        dF = max(-dF_max_brake, min(dF_max, dF));
    else
        dF = max(-dF_max, min(dF_max, dF));
    end
    Fx_total = ctrlState.prevForce + dF;
    ctrlState.prevForce = Fx_total;

    %% ABS
    %% ABS — RMS 5점 안정화 + stoppingDistance 개선을 위한 sweet spot
    kappa_target = 0.132;
    slip = ctrlState.wheelSlip(:);
    absKappaMax = max(abs(slip));
    if absKappaMax > kappa_target
        excess = (absKappaMax - kappa_target) / kappa_target;
        brakeRatioRaw = max(0.74, 1 - 0.38*excess);   % floor↑, gain↓ → RMS 보호
    else
        brakeRatioRaw = 1.0;
    end
    
    % Low-pass filter (더 부드럽게)
    alpha = 0.78;
    ctrlState.brakeRatioFilt = alpha * ctrlState.brakeRatioFilt + (1-alpha) * brakeRatioRaw;
    
    forceCmd.Fx_total   = Fx_total;
    forceCmd.brakeRatio = ctrlState.brakeRatioFilt;
end
