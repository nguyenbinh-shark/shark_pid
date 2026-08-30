function res = verify_shark_vs_pid2(varargin)
%VERIFY_SHARK_VS_PID2  Doi chieu MA C THAT (qua S-Function) voi khoi `PID Controller (2DOF)` cua Simulink.
%
%   res = VERIFY_SHARK_VS_PID2()
%   res = VERIFY_SHARK_VS_PID2('kp',4, 'ki',1.5, 'kd',0.1, 'n',50, 'Ts',0.001)
%   res = VERIFY_SHARK_VS_PID2('out_min',-10, 'out_max',10, 'AntiWindup','clamping')
%   res = VERIFY_SHARK_VS_PID2('out_min',-10, 'out_max',10, 'AntiWindup','back-calculation', 'kb',2.0)
%   res = VERIFY_SHARK_VS_PID2('IntegratorMethod','Backward Euler')
%
%   Script nay tu dong dung mo hinh Simulink, chay song song:
%     1. Khoi S-Function bọc trực tiếp mã C thật (src/shark_pid.c)
%     2. Khoi PID Controller (2DOF) chuan cua Simulink
%   va so sanh hieu ngo ra giua hai ben tren cung tin hieu thu (step + ramp + noise).
%
%   Can: MATLAB R2019a+, Simulink, trinh bien dich C (mex -setup C).
%
%   Tac gia: Tran Nguyen Binh - HUST.

% ---- 1. Duong dan ------------------------------------------------------- %
here   = fileparts(mfilename('fullpath'));
root   = fileparts(fileparts(here));
SRC    = fullfile(root, 'src');       % ma nguon C goc

addpath(here);

% ---- 2. Doc tuy chon va tham so --------------------------------------- %
F_TRAPEZOID_I = uint32(1);
F_CLAMP_I     = uint32(2);
F_BACKCALC_I  = uint32(4);

opt = struct('Ts', 1e-3, 'Tend', 3.0, 'Plot', true, 'Rebuild', false, ...
             'Keep', true, 'Verbose', true, ...
             'IntegratorMethod', 'Trapezoidal', 'AntiWindup', 'clamping');

cfg = struct();
cfg.kp = 4.0;   cfg.ki = 1.5;   cfg.kd = 0.10;
cfg.b  = 0.80;  cfg.c  = 0.00;  cfg.n  = 50.0;
cfg.out_min = -1e6;  cfg.out_max = 1e6;
cfg.i_min   = -1e6;  cfg.i_max   = 1e6;
cfg.kb      = 0.0;
cfg.flags   = bitor(F_TRAPEZOID_I, F_CLAMP_I);

if mod(numel(varargin), 2) ~= 0
    error('Tham so phai di theo cap ten-gia tri.');
end
for k = 1:2:numel(varargin)
    nm = varargin{k};
    val = varargin{k+1};
    if isfield(opt, nm)
        opt.(nm) = val;
    elseif isfield(cfg, nm)
        cfg.(nm) = double(val);
    else
        error('Khong co tuy chon hoac tham so ten "%s".', nm);
    end
end

% Phuong phap tich phan
switch lower(opt.IntegratorMethod)
    case {'trapezoidal', 'trapezoid'}
        cfg.flags = bitor(uint32(cfg.flags), F_TRAPEZOID_I);
        imethod_str = 'Trapezoidal';
    case {'backward euler', 'backwardeuler'}
        cfg.flags = bitand(uint32(cfg.flags), bitcmp(F_TRAPEZOID_I));
        imethod_str = 'Backward Euler';
    otherwise
        error('IntegratorMethod phai la ''Trapezoidal'' hoac ''Backward Euler''.');
end

% Chong windup
cfg.flags = bitand(uint32(cfg.flags), bitcmp(bitor(F_CLAMP_I, F_BACKCALC_I)));
switch lower(opt.AntiWindup)
    case 'clamping'
        cfg.flags = bitor(uint32(cfg.flags), F_CLAMP_I);
        aw_str = 'clamping';
    case 'back-calculation'
        cfg.flags = bitor(uint32(cfg.flags), F_BACKCALC_I);
        aw_str = 'back-calculation';
    case 'none'
        aw_str = 'none';
    otherwise
        error('AntiWindup phai la ''clamping'', ''back-calculation'' hoac ''none''.');
end

Ts = opt.Ts;

% ---- 3. Tu dong bien dich MEX S-Function neu can ----------------------- %
mex_file = fullfile(here, ['shark_pid_sfun.' mexext]);
sfun_c   = fullfile(here, 'shark_pid_sfun.c');
deps     = {sfun_c, fullfile(here, 'shark_pid_sfun_common.h'), ...
            fullfile(SRC, 'shark_pid.c'), fullfile(SRC, 'shark_pid.h')};

if opt.Rebuild || sp_outdated(mex_file, deps)
    fprintf('Bien dich MEX S-Function (loi C moi hon file .mex)... ');
    clear('mex');
    args = {'-outdir', here, ['-I' SRC], ['-I' here], sfun_c, ...
            fullfile(SRC, 'shark_pid.c')};
    if ispc
        mex(args{:});
    else
        mex('CFLAGS=$CFLAGS -std=c99', args{:});
    end
    fprintf('xong.\n');
end

% In thong so neu can
if opt.Verbose
    fprintf('\n=== THONG SO DOI CHIEU SIMULINK ===\n');
    fprintf('  Ts: %g s | P: %g | I: %g | D: %g | N: %g | b: %g | c: %g\n', ...
            Ts, cfg.kp, cfg.ki, cfg.kd, cfg.n, cfg.b, cfg.c);
    fprintf('  Out limits: [%g, %g] | I limits: [%g, %g]\n', ...
            cfg.out_min, cfg.out_max, cfg.i_min, cfg.i_max);
    fprintf('  Integrator: %s | Anti-windup: %s (Kb = %g)\n', ...
            imethod_str, aw_str, cfg.kb);
    fprintf('=====================================\n\n');
end

% ---- 4. Sinh tin hieu thu ---------------------------------------------- %
t = (0:Ts:opt.Tend).';
n_pts = numel(t);

r = zeros(n_pts, 1);
r(t >= 0.2) = 1.0;                              % step
r(t >= 1.2) = 1.0 + 0.5*(t(t >= 1.2) - 1.2);    % ramp

y = 0.6*(1 - exp(-t/0.35)) .* (t >= 0.2);       % measurement gia lap
y = y + 0.002*sin(2*pi*37*t);                   % noise 37 Hz
r(1) = 0; y(1) = 0;

% ---- 5. Dung mo hinh Simulink doi chieu --------------------------------- %
mdl = 'cmp_shark_pid2';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
load_system('simulink');

assignin('base', 'cmp_r',  [t r]);
assignin('base', 'cmp_y',  [t y]);
assignin('base', 'cmp_Ts', Ts);

B = @(nm) [mdl '/' nm];

% Nguon tin hieu vao
add_block('simulink/Sources/From Workspace', B('R'), ...
    'Position', [30 40 140 80], 'VariableName', 'cmp_r', ...
    'SampleTime', 'cmp_Ts', 'Interpolate', 'off', 'ZeroCross', 'off', ...
    'OutputAfterFinalValue', 'Holding final value');
add_block('simulink/Sources/From Workspace', B('Y'), ...
    'Position', [30 130 140 170], 'VariableName', 'cmp_y', ...
    'SampleTime', 'cmp_Ts', 'Interpolate', 'off', 'ZeroCross', 'off', ...
    'OutputAfterFinalValue', 'Holding final value');

% Khoi S-Function goi ma C that
add_block('simulink/User-Defined Functions/S-Function', B('SHARK'), ...
    'Position', [250 150 360 220], 'FunctionName', 'shark_pid_sfun', ...
    'Parameters', 'Ts,kp,ki,kd,b,c,n,out_min,out_max,i_min,i_max,imethod-1,awmethod-1,kb');
make_shark_mask(B('SHARK'), false);
num2s = @(x) sprintf('%.17g', double(x));
set_param(B('SHARK'), ...
    'Ts', num2s(Ts), 'kp', num2s(cfg.kp), 'ki', num2s(cfg.ki), 'kd', num2s(cfg.kd), ...
    'b', num2s(cfg.b), 'c', num2s(cfg.c), 'n', num2s(cfg.n), ...
    'out_min', num2s(cfg.out_min), 'out_max', num2s(cfg.out_max), ...
    'i_min', num2s(cfg.i_min), 'i_max', num2s(cfg.i_max), ...
    'imethod', imethod_str, 'awmethod', aw_str, 'kb', num2s(cfg.kb));

% Khoi PID Controller (2DOF) chuan cua Simulink
add_block('simulink/Continuous/PID Controller (2DOF)', B('PID2'), ...
    'Position', [250 30 360 110]);
sp_set_simulink_pid(B('PID2'), cfg, Ts, imethod_str, aw_str);

% Khoi tinh do lech
add_block('simulink/Math Operations/Sum', B('diff'), ...
    'Position', [450 120 480 150], 'Inputs', '+-');

% Ghi ket qua ra Workspace & Scope
add_block('simulink/Sinks/To Workspace', B('OUT_PID2'), ...
    'Position', [560 30 660 60], 'VariableName', 'u_pid2', ...
    'SaveFormat', 'Array', 'SampleTime', 'cmp_Ts');
add_block('simulink/Sinks/To Workspace', B('OUT_SHARK'), ...
    'Position', [560 170 660 200], 'VariableName', 'u_shark', ...
    'SaveFormat', 'Array', 'SampleTime', 'cmp_Ts');
add_block('simulink/Sinks/Scope', B('Scope'), 'Position', [560 105 610 145]);

% Noi day
add_line(mdl, 'R/1',     'PID2/1',      'autorouting', 'on');
add_line(mdl, 'Y/1',     'PID2/2',      'autorouting', 'on');
add_line(mdl, 'R/1',     'SHARK/1',     'autorouting', 'on');
add_line(mdl, 'Y/1',     'SHARK/2',     'autorouting', 'on');
add_line(mdl, 'PID2/1',  'OUT_PID2/1',  'autorouting', 'on');
add_line(mdl, 'SHARK/1', 'OUT_SHARK/1', 'autorouting', 'on');
add_line(mdl, 'PID2/1',  'diff/1',      'autorouting', 'on');
add_line(mdl, 'SHARK/1', 'diff/2',      'autorouting', 'on');
add_line(mdl, 'diff/1',  'Scope/1',     'autorouting', 'on');

% Cau hinh Solver Fixed-step Discrete
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'cmp_Ts', 'StopTime', num2str(t(end), '%.17g'), ...
    'SaveFormat', 'Array', 'ReturnWorkspaceOutputs', 'on');

% ---- 6. Chay mo phong -------------------------------------------------- %
out     = sim(mdl);
u_pid2  = out.u_pid2(:);
u_shark = out.u_shark(:);

m = min(numel(u_pid2), numel(u_shark));
t = t(1:m); u_pid2 = u_pid2(1:m); u_shark = u_shark(1:m);

% ---- 7. Danh gia sai so ------------------------------------------------ %
scale = max(abs(u_pid2));
if ~(scale > 0), scale = 1; end

err_c = max(abs(u_shark - u_pid2)) / scale;
TOL_C = 1e-5;   % San lam tron float (C) vs double (Simulink)

fprintf('\n=== KET QUA MO PHONG (%d nhip, Ts = %g s) ===\n', m, Ts);
fprintf('  Bien do max |u|max                     : %.6g\n', scale);
fprintf('  Sai so lon nhat (C S-Function vs PID2) : %.3e  (nguong %.0e)\n', err_c, TOL_C);

ok = err_c < TOL_C;
if ok
    fprintf('\n  >> [DAT] shark_pid.c (S-Function) KHOP HOAN TOAN voi khoi PID (2DOF) Simulink!\n');
else
    fprintf('\n  >> [LECH] Phat hien sai lech giua ma C va khoi PID Simulink.\n');
end
fprintf('  Mo hinh: %s (dong bang close_system(''%s'', 0))\n\n', mdl, mdl);

% ---- 8. Ve do thi ------------------------------------------------------ %
if opt.Plot
    figure('Name', 'shark_pid S-Function vs Simulink PID (2DOF)');
    subplot(2,1,1);
    plot(t, u_pid2, 'LineWidth', 1.4); hold on;
    plot(t, u_shark, '--', 'LineWidth', 1.2);
    legend('Simulink PID (2DOF)', 'shark\_pid.c (S-Function)', 'Location', 'best');
    ylabel('u'); grid on; title('Ngo ra dieu khien u(t)');
    
    subplot(2,1,2);
    semilogy(t, abs(u_shark - u_pid2) + eps, 'r', 'LineWidth', 1.2); grid on;
    xlabel('t [s]'); ylabel('|u_{shark} - u_{pid2}|'); title('Sai so tuyet doi giua S-Function va Khoi Simulink');
end

if opt.Keep
    open_system(mdl);
else
    close_system(mdl, 0);
end

res = struct('t', t, 'r', r(1:m), 'y', y(1:m), ...
             'u_pid2', u_pid2, 'u_shark', u_shark, ...
             'err_max', err_c, 'pass', ok, 'cfg', cfg, 'model', mdl);
end

% ========================================================================== %
% Cac ham tien ich noi bo
% ========================================================================== %


function sp_set_simulink_pid(blk, cfg, Ts, imethod_str, aw_str)
num2s = @(x) sprintf('%.17g', double(x));

set_param(blk, ...
    'Form', 'Parallel', ...
    'TimeDomain', 'Discrete-time', ...
    'SampleTime', num2s(Ts), ...
    'P', num2s(cfg.kp), ...
    'I', num2s(cfg.ki), ...
    'D', num2s(cfg.kd), ...
    'b', num2s(cfg.b), ...
    'c', num2s(cfg.c), ...
    'IntegratorMethod', imethod_str, ...
    'LimitOutput', 'on', ...
    'LowerSaturationLimit', num2s(cfg.out_min), ...
    'UpperSaturationLimit', num2s(cfg.out_max), ...
    'LimitIntegrator', 'on', ...
    'LowerIntegratorSaturationLimit', num2s(cfg.i_min), ...
    'UpperIntegratorSaturationLimit', num2s(cfg.i_max));

% Phuong phap chong windup trong mask Simulink co ten la 'AntiWindupMode'
try
    set_param(blk, 'AntiWindupMode', aw_str);
catch
    try
        set_param(blk, 'AntiWindupMethod', aw_str);
    catch
    end
end

if cfg.n > 0
    try set_param(blk, 'UseFilteredDerivative', 'on'); catch; end
    try set_param(blk, 'FilterMethod', 'Backward Euler'); catch; end
    try set_param(blk, 'N', num2s(cfg.n)); catch; end
else
    try set_param(blk, 'UseFilteredDerivative', 'off'); catch; end
end

if strcmp(aw_str, 'back-calculation')
    try set_param(blk, 'Kb', num2s(cfg.kb)); catch; end
end
end

function tf = sp_outdated(target, deps)
d = dir(target);
if isempty(d), tf = true; return; end
tf = false;
for k = 1:numel(deps)
    dk = dir(deps{k});
    if ~isempty(dk) && dk.datenum > d.datenum
        tf = true;
        return;
    end
end
end
