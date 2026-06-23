function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC) — v3 튜닝본
%   v3: AFS 약화(수렴 우선) + A4 저속 ESC 완전 차단 + β-limiter 보수화

    if ~isfield(ctrlState,'intError');   ctrlState.intError  = 0; end
    if ~isfield(ctrlState,'prevError');  ctrlState.prevError = 0; end

    %% (1) AFS : yaw-rate 추종 (부드럽게, 수렴 우선)
    err = yawRateRef - yawRate;

    v_ref = 20;
    s_afs = max(0.4, min(1.0, v_ref / max(vx, 5)));

    % 게인 대폭 하향 — 과도응답/오버슈트 방지, 안정성 우선
    Kp = 0.35 * CTRL.LAT.Kp * s_afs;
    Ki = 0.10 * CTRL.LAT.Ki * s_afs;
    Kd = 0.80 * CTRL.LAT.Kd * s_afs;

    % 강한 washout (정상상태 AFS→0, A4 보호)
    leak = 2.0;
    ctrlState.intError = (1 - leak*dt) * ctrlState.intError + err * dt;
    ctrlState.intError = max(-CTRL.LAT.intMax, min(CTRL.LAT.intMax, ctrlState.intError));

    dErr = (err - ctrlState.prevError) / max(dt, 1e-6);
    ctrlState.prevError = err;

    steerAFS = Kp*err + Ki*ctrlState.intError + Kd*dErr;

    steerAFS = steerAFS * max(0, min(1.0, (vx - 12) / 6));

    % AFS 권한 제한 — 경로추종 방해 최소화 (15%)
    afsLimit = 0.15 * LIM.MAX_STEER_ANGLE;
    steerAFS = max(-afsLimit, min(afsLimit, steerAFS));

    %% (2) ESC : β-limiter (고속에서만, 보수적)
    beta_th = deg2rad(4.0);          % 3.0 → 4.0 : 정상선회 작은 β 무시
    Kbeta   = 6.0e4;
    s_esc = max(0, min(1.0, (vx - 12) / 6));
    yawMoment = 0;
    absBeta = abs(slipAngle);
    % β 변화율이 작으면(정상상태 선회) ESC 개입 차단
    if ~isfield(ctrlState,'prevBeta'); ctrlState.prevBeta = slipAngle; end
    betaRate = abs(slipAngle - ctrlState.prevBeta) / max(dt,1e-6);
    ctrlState.prevBeta = slipAngle;
    transient = min(1.0, betaRate / deg2rad(5.0));   % 5 deg/s 이상이면 full
    if absBeta > beta_th && s_esc > 0
        yawMoment = -Kbeta * sign(slipAngle) * (absBeta - beta_th) * s_esc * transient;
    end

    Mmax = 4000;
    yawMoment = max(-Mmax, min(Mmax, yawMoment));

    deltaAdd.steerAngle = steerAFS;
    deltaAdd.yawMoment  = yawMoment;
end
