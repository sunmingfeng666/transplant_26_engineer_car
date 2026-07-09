% =========================================================================
% engineer_arm.m  —— 工程车 6-DOF 机械臂 MATLAB 联调上位机
%
% 【配合固件】board2 Arm_MatlabDebug 模块（编译开关 ARM_MATLAB_DEBUG_ENABLE=1）
%
% 【通信协议】UART7 @ 115200，VOFA+ JustFloat 双向
%   下发 MATLAB → MCU : single([J1 J2 J3 J4 J5 J6, Inf])  共 6 通道 + Inf 帧尾
%                        MCU 端当前仅采用 J2/J4/J5，其余通道预留。
%   回传 MCU → MATLAB : 20 通道，ch0-5 = J1..J6 实际位置(rad)，用于 3D 显示；
%                        ch19 = 链路在线标志（1=在线，0=离线/未编联调模块）。
%
% 【单位】关节角一律弧度(rad)。
%
% 【DH 参数】下方为占位值，上车前需按实车测量填写（见 TODO 标注），否则 3D 姿态不准。
%
% 【依赖】Peter Corke Robotics Toolbox（Link / SerialLink / plot）
%
% 【安全提示】
%   1. 首次联调前机械支撑好整臂，确认所有关节在线。
%   2. 固件端 Arm_MatlabDebug_Enable 默认 0，需手动设 1 才注入目标。
%   3. 单轴小幅度验证方向/零位正确后，再多轴联动。
%   4. 链路掉线 200ms 自动回退 DBUS 控制，不会悬停在危险位置。
% =========================================================================
clear; clc; close all;

% ==========================================
% 1. 工程臂 6-DOF 运动学模型（DH 占位，上车按实车改）
%    TODO: 按工程臂实际 DH 参数填写 theta/d/a/alpha/offset/qlim。
%          下面仅为可运行占位，用于验证通信链路与显示框架，3D 姿态不准。
% ==========================================
L1 = 15.0; L2 = 25.0; L3 = 20.0; L4 = 10.0;   % TODO 占位连杆尺寸(cm)

r(1) = Link([0 L1  0   pi/2], 'standard');    % J1 基座旋转
r(2) = Link([0 0   L2  0   ], 'standard');    % J2 大臂俯仰
r(3) = Link([0 0   L3  0   ], 'standard');    % J3 小臂俯仰
r(4) = Link([0 0   0   pi/2], 'standard');    % J4 腕部俯仰
r(5) = Link([0 L4  0  -pi/2], 'standard');    % J5 腕部旋转
r(6) = Link([0 0   0   0   ], 'standard');    % J6 末端旋转

% J2/J4/J5 联调轴的限位（rad）——与固件 Arm_Ctrl.c 中的钳位一致，供滑块范围使用。
J2_MIN = -0.532349;  J2_MAX = 3.67875862;
J4_MIN = -1.8297427; J4_MAX = 1.848;
J5_MIN = -1.76146317;J5_MAX = 1.67410;

robot = SerialLink(r, 'name', 'Engineer_6DOF');

% ==========================================
% 2. 串口初始化（VOFA+ JustFloat，115200 波特率）
%    TODO: COM_PORT 改为板子实际串口号。
% ==========================================
COM_PORT  = "COM20";    % TODO: 改成板子对应的 COM 口
BAUD_RATE = 115200;

try
    s = serialport(COM_PORT, BAUD_RATE);
    flush(s);
    disp('串口连接成功！');
catch ME
    disp(['串口连接失败: ', ME.message]);
    disp('切换为纯本地显示模式（不下发、不接收）。');
    s = [];
end
cleanupObj = onCleanup(@() cleanupSerial(s));

% ==========================================
% 3. GUI：J2/J4/J5 三滑块下发 + 实际姿态 3D 显示
%    - cmd_pose：MATLAB 下发目标，6 维(rad)，J1/J3/J6 先固定 0（本期不联调）。
%    - fb_pose：MCU 回传实际关节角，6 维(rad)，取自 UART7 回传帧 ch0-5。
% ==========================================
fig = figure('Name', 'Engineer Arm MATLAB Debug', 'Position', [100, 100, 1000, 720]);
ax = axes(fig, 'Position', [0.05, 0.1, 0.62, 0.8]);

cmd_pose = zeros(1, 6);  % 下发目标关节角(rad)
fb_pose  = zeros(1, 6);  % MCU 回传实际关节角(rad)

robot.plot(cmd_pose, 'workspace', [-80 80 -80 80 -20 80]);

panel = uipanel(fig, 'Title', '联调控制面板', 'Position', [0.70, 0.05, 0.28, 0.9]);

uicontrol(panel, 'Style', 'text', 'String', '下发关节角（rad）：J2 / J4 / J5', ...
    'Units', 'normalized', 'Position', [0.05, 0.92, 0.9, 0.05], 'HorizontalAlignment', 'left');

% 三个联调轴配置：{显示名称, 6 维下标, 限位下限, 限位上限}
axes_cfg = {'J2', 2, J2_MIN, J2_MAX; 'J4', 4, J4_MIN, J4_MAX; 'J5', 5, J5_MIN, J5_MAX};
sliders = gobjects(1, 3);
labels  = gobjects(1, 3);
for i = 1:3
    y = 0.80 - (i-1)*0.16;
    labels(i) = uicontrol(panel, 'Style', 'text', 'String', axes_cfg{i,1}, ...
        'Units', 'normalized', 'Position', [0.05, y+0.05, 0.9, 0.04], 'HorizontalAlignment', 'left');
    sliders(i) = uicontrol(panel, 'Style', 'slider', ...
        'Min', axes_cfg{i,3}, 'Max', axes_cfg{i,4}, 'Value', 0, ...
        'Units', 'normalized', 'Position', [0.05, y, 0.9, 0.04]);
end

% 链路状态指示：显示 MCU 回传帧 ch19（链路在线标志），1=在线，0=离线/未编联调模块。
link_txt = uicontrol(panel, 'Style', 'text', 'String', '链路: --', ...
    'Units', 'normalized', 'Position', [0.05, 0.20, 0.9, 0.05], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold');

% ==========================================
% 4. 实时主循环 @ 20Hz（下发 MATLAB 目标 + 更新 3D 显示）
%    - 每 50ms 读一次滑块值，封包 JustFloat 帧下发。
%    - 后台线程收到 MCU 回传帧（20 通道）时自动解析 ch0-5 为实际关节角，
%      主循环只负责读解析结果并刷新 3D plot。
% ==========================================
vofa_tail = uint8([0, 0, 128, 127]);   % JustFloat 帧尾（float +Inf 小端）
rx_buffer = uint8([]);  % 接收缓冲区，用于逐字节匹配帧尾

while ishandle(fig)
    % ==========================================
    % A. 接收 MCU 回传帧（20 通道 JustFloat）：取 ch0-5 为实际关节角，ch19 为链路状态
    % ==========================================
    if ~isempty(s) && s.NumBytesAvailable > 0
        try
            new_bytes = reshape(read(s, s.NumBytesAvailable, "uint8"), 1, []);
            rx_buffer = [rx_buffer, new_bytes];
            % 防缓冲区无限增长：超 4KB 时裁到最近 2KB
            if length(rx_buffer) > 4096
                rx_buffer = rx_buffer(end-2047:end);
            end
            % 滑动窗口查找所有帧尾，取最后一帧（最新）
            tail_idx = strfind(rx_buffer, vofa_tail);
            if ~isempty(tail_idx)
                last_tail = tail_idx(end);
                % 帧起点 = 上一个帧尾 + 4，若无上一个则从缓冲区头开始
                if length(tail_idx) > 1
                    start_i = tail_idx(end-1) + 4;
                else
                    start_i = 1;
                end
                frame = rx_buffer(start_i : last_tail-1);
                % 帧长必须 4 字节对齐（20 通道 = 80 字节）
                if ~isempty(frame) && mod(length(frame), 4) == 0
                    vals = double(typecast(frame, 'single'));
                    if length(vals) >= 6
                        fb_pose = vals(1:6);   % ch0-5 = J1..J6 实际位置(rad)
                        if length(vals) >= 20
                            online = vals(20);  % ch19 = 链路在线标志（1=在线，0=离线/未编联调模块）
                            set(link_txt, 'String', sprintf('链路: %s', ...
                                ternary(online>0.5, '在线', '离线')));
                        end
                    end
                end
                % 清除已解析数据，保留帧尾后的残留字节（可能是下一帧的开头）
                rx_buffer = rx_buffer(last_tail + 4 : end);
            end
        catch ME
            disp(['串口处理异常: ', ME.message]);
            flush(s); rx_buffer = uint8([]);
        end
    end

    % ==========================================
    % B. 读取滑块值，组装下发目标关节角（6 维，J1/J3/J6 保持 0）
    % ==========================================
    for i = 1:3
        idx = axes_cfg{i, 2};
        cmd_pose(idx) = get(sliders(i), 'Value');
        set(labels(i), 'String', sprintf('%s: %.3f rad', axes_cfg{i,1}, cmd_pose(idx)));
    end

    % ==========================================
    % C. 3D 显示：优先显示 MCU 回传的实际姿态（fb_pose），无串口时显示下发目标（cmd_pose）
    % ==========================================
    if ~isempty(s)
        robot.animate(fb_pose);
    else
        robot.animate(cmd_pose);
    end

    % ==========================================
    % D. 下发 JustFloat 帧：single([J1..J6, Inf])，共 7 个 float（28 字节）
    %    MCU 端解析后取 J2/J4/J5 注入控制器，其余关节保持 DBUS 逻辑。
    % ==========================================
    if ~isempty(s)
        write(s, single([cmd_pose, Inf]), "single");
    end

    drawnow;  % 刷新 GUI，响应滑块拖动和窗口关闭
end
disp('程序结束。');

% ==========================================
% 辅助函数
% ==========================================
% 三元运算符：替代 cond ? a : b
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

% 串口清理回调：关闭窗口时自动释放串口资源
function cleanupSerial(serialObj)
    if ~isempty(serialObj) && isvalid(serialObj)
        clear serialObj;
        disp('串口已安全关闭。');
    end
end
