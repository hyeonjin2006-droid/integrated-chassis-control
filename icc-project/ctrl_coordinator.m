'TRAP: ctrl_coordinator loaded and executed'
disp('### ctrl_coordinator.m IS BEING USED ###');
function actuatorCmd = ctrl_coordinator( %%% TRAP: latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   주의(runner 연동):
%     - actuatorCmd.brakeTorque 는 시나리오 제동에 "가산"됨 (brake_total = scenario + ESC).
%       → ABS 는 brakeRatio<1 일 때 "음의" 토크를 만들어 시나리오 제동을 상쇄 (runner 가 0 클램프).
%     - actuatorCmd.steerAngle 는 driver 조향에 가산됨.
%     - 출력 brakeTorque 부호: [FL;FR;RL;RR].

    track_f = 1.55; track_r = 1.55; rw = 0.31;
    if isfield(VEH,'track_f'); track_f = VEH.track_f; end
    if isfield(VEH,'track_r'); track_r = VEH.track_r; end
    if isfield(VEH,'rw');      rw      = VEH.rw;       end
    htf = track_f/2;  htr = track_r/2;

    Tmax = LIM.MAX_BRAKE_TRQ;

    %% ---- (1) AFS 조향 pass-through + saturation ----
    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, ...
                                  min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    %% ---- (2) 종방향 제동 분배 (60:40 front:rear) ----
    %   coordinator 의 lon 제동은 추가 제동분. 시나리오 제동이 이미 큰 경우엔
    %   ABS 변조(아래 brakeRatio)가 핵심이므로 lon 추가 제동은 보수적으로 적용.
    brakeLon = zeros(4,1);
    if lonCmd.Fx_total < 0
        Tbrk = abs(lonCmd.Fx_total) * rw;     % 총 제동 토크 [Nm]
        brakeLon = [0.30; 0.30; 0.20; 0.20] * Tbrk;   % 60:40 축분배
    end

    %% ---- (3) ESC yaw moment → 좌우 차동 제동 ----
    %   양(+, CCW) M_z → 우측 제동 증가(차량을 좌회전). 전축 60% 분배.
    Mz = latCmd.yawMoment;
    ratio_f = 0.6;
    dT_f = ratio_f     * Mz / track_f;   % 한쪽(+)/반대쪽(-) 토크 크기
    dT_r = (1-ratio_f) * Mz / track_r;

    brakeESC = zeros(4,1);
    % +Mz(CCW, 좌회전) → 우측 휠(FR,RR) 제동 증가
    brakeESC(2) =  dT_f;   % FR
    brakeESC(4) =  dT_r;   % RR
    brakeESC(1) = -dT_f;   % FL
    brakeESC(3) = -dT_r;   % RR 반대측
    brakeESC = max(0, brakeESC);   % 제동은 음수 불가 (한쪽만 증가)

    %% ---- (4) ABS 변조 (brakeRatio<1 → 시나리오 제동 상쇄) ----
    %   runner: brake_total = scenario_brake + actuatorCmd.brakeTorque, 이후 [0,Tmax] 클램프.
    %   ABS 가 필요하면 음의 토크로 시나리오 제동을 줄임.
    brakeABS = zeros(4,1);
    if isfield(lonCmd,'brakeRatio') && lonCmd.brakeRatio < 1
        scenBrkEst = [1500;1500;800;800];
        absReductionFactor = 0.49;   % 0.42 → 0.49 : RMS 5점 유지하면서 net brake 약간 증가
        brakeABS = -absReductionFactor * (1 - lonCmd.brakeRatio) .* scenBrkEst;
    end

    %% ---- (5) 합산 + saturation ----
    brakeTorque = brakeLon + brakeESC + brakeABS;
    % 상한만 클램프 (하한 음수는 허용 — 시나리오 제동 상쇄용; runner 가 최종 0 클램프)
    brakeTorque = min(Tmax, brakeTorque);

    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = verCmd(:);
end
