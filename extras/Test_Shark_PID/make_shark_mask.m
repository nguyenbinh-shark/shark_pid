function make_shark_mask(blk, show_dialog)
if nargin < 2, show_dialog = true; end
%MAKE_SHARK_MASK  Tao mask cho khoi S-Function shark_pid_sfun -> chinh tham so
%                 bang dialog thay vi go chuoi so vao truong "Parameters".
%
%  Cach hoat dong: truong Parameters cua S-Function tro toi TEN BIEN
%  (Ts, kp, ki, ...). Cac ten nay duoc giai quyet trong mask workspace, nen
%  nguoi dung chi thay dialog dep, S-function van nhan gia tri so.
%
%  Hai o combo box (Integrator method, Anti-windup method) la popup co
%  Evaluate = on, tra ve CHI SO tinh tu 1, nen chuoi Parameters tru bot 1 cho
%  khop ma so trong shark_pid_sfun_common.h.
%
%  Dung duoc theo 2 cach:
%    make_shark_mask('cmp_shark_pid2/SHARK')   % mask khoi da co
%    make_shark_mask()                          % tao 1 model thu vien rieng
%
%  Tac gia: Tran Nguyen Binh - HUST.

if nargin < 1 || isempty(blk)
    % --- tao model thu vien chua 1 khoi masked de tai su dung ---
    mdl = 'shark_pid_lib';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl);
    open_system(mdl);
    blk = [mdl '/SharkPID'];
    add_block('simulink/User-Defined Functions/S-Function', blk, ...
              'Parameters', 'Ts,kp,ki,kd,b,c,n,out_min,out_max,i_min,i_max,imethod-1,awmethod-1,kb', ...
              'FunctionName','shark_pid_sfun');
end

% --- 1. Xoa mask cu neu co, tao lai ---
old = Simulink.Mask.get(blk);
if ~isempty(old), old.delete; end
m = Simulink.Mask.create(blk);

m.Type        = 'shark_pid';
m.Description = ['Bo dieu khien PID nhung shark_pid (loi C99).' newline ...
                 'Tuong duong khoi PID (2DOF) khi:' newline ...
                 '  P=kp  I=ki  D=kd  N=n  b=b  c=c' newline ...
                 '  Filter method = Backward Euler'];

% --- 2. Truong Parameters tro toi ten bien, dung thu tu enum trong file C ---
%     popup tra ve chi so tinh tu 1 -> tru 1 de ve ma so cua file C.
set_param(blk,'Parameters', ...
    ['Ts,kp,ki,kd,b,c,n,out_min,out_max,i_min,i_max,' ...
     'imethod-1,awmethod-1,kb']);



% --- 3. Cac o nhap ------------------------------------------------------
p = { ...
 'Ts',      'Sample time Ts [s]',                    '1e-3'
 'kp',      'kp  (= P)',                             '4.0'
 'ki',      'ki  [1/s]  (= I)',                      '1.5'
 'kd',      'kd  [s]    (= D)',                      '0.10'
 'b',       'Setpoint weight b  (khau P)',           '1.0'
 'c',       'Setpoint weight c  (khau D)',           '0.00'
 'n',       'Filter coefficient N [rad/s]  (= o N)', '100'
 'out_min', 'Gioi han ngo ra: min',                  '-1e6'
 'out_max', 'Gioi han ngo ra: max',                  '1e6'
 'i_min',   'Gioi han khau I: min',                  '-1e6'
 'i_max',   'Gioi han khau I: max',                  '1e6'
 'kb',      'Kb [1/s] (chi dung voi back-calculation)', '0'  };

for k = 1:size(p,1)
    m.addParameter('Type','edit', 'Name',p{k,1}, ...
                   'Prompt',p{k,2}, 'Value',p{k,3});
end

% --- 4. Hai o combo box, dung dung ten cua khoi PID (2DOF) --------------
m.addParameter('Type','popup', 'Name','imethod', ...
    'Prompt','Integrator method', 'Evaluate','on', ...
    'TypeOptions',{'Backward Euler','Trapezoidal'}, 'Value','Trapezoidal');
m.addParameter('Type','popup', 'Name','awmethod', ...
    'Prompt','Anti-windup method', 'Evaluate','on', ...
    'TypeOptions',{'none','clamping','back-calculation'}, 'Value','clamping');

% --- 5. Gom nhom cho dialog de nhin ------------------------------------
m.addDialogControl('Type','group','Name','grpGain','Prompt','He so / Gains');
m.addDialogControl('Type','group','Name','grpLim' ,'Prompt','Gioi han / Limits');
m.addDialogControl('Type','group','Name','grpMeth','Prompt','Phuong phap / Methods');
try
    for nm = {'kp','ki','kd','b','c','n'}
        m.getDialogControl(nm{1}).moveTo('grpGain');
    end
    for nm = {'out_min','out_max','i_min','i_max','kb'}
        m.getDialogControl(nm{1}).moveTo('grpLim');
    end
    for nm = {'imethod','awmethod'}
        m.getDialogControl(nm{1}).moveTo('grpMeth');
    end
catch
    % moveTo khong co o ban MATLAB cu -> bo qua, dialog van dung
end

% --- 6. Mat khoi -------------------------------------------------------
m.Display = [ ...
 'port_label(''input'',1,''r'');' newline ...
 'port_label(''input'',2,''y'');' newline ...
 'port_label(''output'',1,''u'');' newline ...
 'disp(sprintf(''shark\\_pid\nPID 2DOF''));'];

fprintf('Da tao mask cho %s.\n', blk);
if show_dialog
    open_system(blk,'mask');
end
end
