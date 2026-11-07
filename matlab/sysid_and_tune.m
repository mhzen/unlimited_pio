function sysid_and_tune(csv_path)
% sysid_and_tune.m
% Baca CSV dari aplikasi pid-gather (download http://192.168.4.1/csv),
% identifikasi plant per roda, hitung gain PID, lalu cetak blok konstanta
% yang bisa langsung di-tempel ke include/config/drive_config.h.
%
% Pemakaian:
%   >> sysid_and_tune              % pakai data.csv di folder ini
%   >> sysid_and_tune('file.csv')  % pakai file lain
%
% Kalau data.csv tidak ada, script memakai file pid_data_*.csv terbaru di
% folder ini (nama default dari endpoint /csv).
%
% Kolom yang diharapkan (nama harus sama, urutan bebas):
%   dt_ms, encLF..RB, pwmLF..RB, tgtCpsLF..RB, measCpsLF..RB
% Baris komentar yang diawali '#' diabaikan.

close all;

script_dir = fileparts(mfilename('fullpath'));

%% Tentukan file input
if nargin < 1 || isempty(csv_path)
    csv_path = fullfile(script_dir, 'data.csv');
    if ~isfile(csv_path)
        listing = dir(fullfile(script_dir, 'pid_data_*.csv'));
        if ~isempty(listing)
            [~, newest] = max([listing.datenum]);
            csv_path = fullfile(script_dir, listing(newest).name);
            fprintf('data.csv tidak ada, memakai %s\n', listing(newest).name);
        end
    end
end

if ~isfile(csv_path)
    error(['File tidak ditemukan: %s\n' ...
           'Download /csv dari robot lalu simpan sebagai %s.'], ...
          csv_path, fullfile(script_dir, 'data.csv'));
end

fprintf('Memuat %s ...\n', csv_path);

%% Metadata dari baris komentar '# ...' (mode/axis/amplitude/use_pid)
fid = fopen(csv_path, 'r');
first_line = fgetl(fid);
fclose(fid);
if ischar(first_line) && startsWith(strtrim(first_line), '#')
    metadata = strtrim(first_line);
    fprintf('Metadata: %s\n', metadata);
    if contains(metadata, 'use_pid=1')
        warning(['Data ini direkam closed-loop (use_pid=1); hasil ' ...
                 'identifikasi plant akan bias. Pakai data open-loop.']);
    end
    if contains(metadata, 'truncated=1')
        warning('CSV terpotong (truncated=1); tambah kMaxCsvRows.');
    end
end

%% Muat data (lewati baris komentar)
data = readtable(csv_path, 'CommentStyle', '#');

required = {'dt_ms', ...
            'encLF', 'encRF', 'encLB', 'encRB', ...
            'pwmLF', 'pwmRF', 'pwmLB', 'pwmRB', ...
            'tgtCpsLF', 'tgtCpsRF', 'tgtCpsLB', 'tgtCpsRB', ...
            'measCpsLF', 'measCpsRF', 'measCpsLB', 'measCpsRB'};
missing = required(~ismember(required, data.Properties.VariableNames));
if ~isempty(missing)
    error('Kolom berikut tidak ada di %s: %s', csv_path, strjoin(missing, ', '));
end

% Bangun vektor waktu dari dt_ms (lebih akurat daripada jam dinding)
dt_ms = data.dt_ms;
t = cumsum(dt_ms) / 1000;          % detik dari sampel pertama
Ts = median(dt_ms) / 1000;         % periode sampel nominal
fprintf('Sampel: %d, Ts = %.4f s, durasi = %.1f s\n', ...
    height(data), Ts, t(end) - t(1));

%% Ekstrak sinyal per roda
wheel_names = {'LF', 'RF', 'LB', 'RB'};
n_wheels = length(wheel_names);

% Open-loop: PWM adalah input plant, kecepatan terukur adalah output.
input_cols  = {'pwm', 'pwm', 'pwm', 'pwm'};
output_cols = {'measCps', 'measCps', 'measCps', 'measCps'};

% --- PENGATURAN ORDE SISTEM ---
num_poles = 3;   % ubah untuk orde 3, 4, 5, dst.
num_zeros = 0;   % biasanya 0, maksimal (num_poles - 1)

fprintf('\n========================================\n');
fprintf('Identifikasi sistem & tuning PID per roda\n');
fprintf('========================================\n');

gains = nan(n_wheels, 4);  % Kp, Ki, Kd, Tf

for w = 1:n_wheels
    name = wheel_names{w};
    fprintf('\n--- Roda %s ---\n', name);

    u_raw = data.(sprintf('%s%s', input_cols{w}, name));
    y_raw = data.(sprintf('%s%s', output_cols{w}, name));

    % Baseline hanya dari bagian sebelum eksitasi. Data pid-gather mencatat
    % fase tunggu, jadi normalnya ada segmen nol di awal; kalau tidak ada,
    % baseline dibuat nol supaya step tetap terlihat.
    u_scale = max(abs(u_raw));
    if u_scale < 1e-9
        u_scale = 1;
    end
    first_active = find(abs(u_raw) > 0.02 * u_scale, 1, 'first');
    if isempty(first_active)
        first_active = numel(u_raw) + 1;
    end

    n_baseline = min(first_active - 1, round(2 / Ts));
    if n_baseline >= 10
        u0 = mean(u_raw(1:n_baseline));
        y0 = mean(y_raw(1:n_baseline));
    else
        u0 = 0;
        y0 = 0;
        n_baseline = 0;
    end

    u = u_raw - u0;
    y = y_raw - y0;

    skip = n_baseline + 1;
    u = u(skip:end);
    y = y(skip:end);

    if numel(u) < 50
        error('Data roda %s terlalu pendek setelah baseline (%d sampel).', ...
              name, numel(u));
    end

    % Identifikasi plant: PWM -> CPS terukur
    if exist('iddata', 'file') == 2 && exist('tfest', 'file') == 2
        fprintf('  Memakai System Identification Toolbox...\n');

        idd = iddata(y, u, Ts);
        idd.TimeUnit = 'seconds';
        idd.InputName = {'PWM'};
        idd.OutputName = {'CPS'};

        idd = detrend(idd);

        try
            fprintf('  Estimasi Transfer Function Orde-%d...\n', num_poles);
            G = tfest(idd, num_poles, num_zeros);

            % Pakai report dari model (bukan output kedua compare, yang
            % tipenya beda antar versi MATLAB).
            fprintf('  Fit orde-%d berhasil. Akurasi Fit: %.2f%%\n', ...
                num_poles, G.Report.Fit.FitPercent);
            display(G);
        catch err
            warning('tfest orde-%d gagal (%s). Fallback ke orde 1...', ...
                num_poles, err.message);
            G = tfest(idd, 1, 0);
        end

        % Validasi model
        figure('Name', sprintf('Roda %s - Validasi Model', name));
        compare(idd, G);
        title(sprintf('Roda %s - fit model', name));
        grid on;

        % Tuning PID
        fprintf('  Tuning PID...\n');
        opts = pidtuneOptions('PhaseMargin', 60, 'DesignFocus', 'balanced');
        [C, info] = pidtune(G, 'PIDF', opts);

        gains(w, :) = [C.Kp, C.Ki, C.Kd, C.Tf];
        fprintf('  Kp=%.4f  Ki=%.4f  Kd=%.4f  Tf=%.4f\n', ...
            C.Kp, C.Ki, C.Kd, C.Tf);
        fprintf('  Crossover: %.2f rad/s  PM: %.1f deg\n', ...
            info.CrossoverFrequency, info.PhaseMargin);

        % Respons step loop tertutup
        CL = feedback(C * G, 1);
        figure('Name', sprintf('Roda %s - Step Loop Tertutup', name));
        step(CL);
        title(sprintf('Roda %s - respons step loop tertutup', name));
        grid on;

        % Bode
        figure('Name', sprintf('Roda %s - Bode', name));
        bode(G, 'b', C * G, 'r', CL, 'g');
        legend('Plant G', 'Loop terbuka C*G', 'Loop tertutup');
        title(sprintf('Roda %s - respons frekuensi', name));
        grid on;
    else
        % Fallback: tanpa System ID Toolbox -> Ziegler-Nichols step response
        fprintf(['  System ID Toolbox tidak ditemukan. Memakai step-response ' ...
                 'Ziegler-Nichols...\n']);

        [Kp, Ki, Kd] = zieglerNicholsStep(y, u, Ts, name);
        gains(w, :) = [Kp, Ki, Kd, 0];
    end
end

%% Tabel ringkasan
fprintf('\n========================================\n');
fprintf('Ringkasan Gain PID\n');
fprintf('========================================\n');
fprintf('%-6s %10s %10s %10s %10s\n', 'Wheel', 'Kp', 'Ki', 'Kd', 'Tf');
fprintf('%-6s %10s %10s %10s %10s\n', '-----', '----------', '----------', ...
    '----------', '----------');
for w = 1:n_wheels
    fprintf('%-6s %10.4f %10.4f %10.4f %10.4f\n', ...
        wheel_names{w}, gains(w,1), gains(w,2), gains(w,3), gains(w,4));
end

if any(isnan(gains(:)))
    error('Sebagian roda tidak mendapat gain; periksa data sebelum menyalin.');
end

% Simpan gain untuk dipakai nanti
save(fullfile(script_dir, 'pid_gains.mat'), 'gains', 'wheel_names');
fprintf('\nGain tersimpan di %s\n', fullfile(script_dir, 'pid_gains.mat'));

%% Blok konstanta siap tempel (komentar memakai // agar utuh di C++)
fprintf('\n// --- tempel ke include/config/drive_config.h ---\n');
fprintf('// Sumber: %s\n', csv_path);
fprintf('// Urutan: LF, RF, LB, RB\n');
print_gain_array('kWheelKp', gains(:, 1));
print_gain_array('kWheelKi', gains(:, 2));
print_gain_array('kWheelKd', gains(:, 3));
fprintf('// Tf (filter derivatif; firmware belum memakainya): %s\n', ...
    mat2str(gains(:, 4)', 4));

end

%% Helper: cetak satu array constexpr float[4]
function print_gain_array(name, values)
    parts = compose('%.4ff', values(:).');
    fprintf('constexpr float %s[4] = {%s};\n', name, ...
        strjoin(cellstr(parts), ', '));
end

%% Helper: metode step-response Ziegler-Nichols
function [Kp, Ki, Kd] = zieglerNicholsStep(y, u, Ts, wheel_name)
    % Fit first-order-plus-dead-time (FOPDT): K * exp(-theta*s) / (tau*s + 1)
    % dari step response, lalu terapkan aturan tuning Z-N.

    % Cari tepi step: saat |u| melewati ambang
    uthresh = 0.1 * max(abs(u));
    step_idx = find(abs(u) > uthresh, 1, 'first');
    if isempty(step_idx)
        warning('Tidak ada step terdeteksi untuk roda %s. Memakai gain default.', ...
            wheel_name);
        Kp = 0.1;
        Ki = 0;
        Kd = 0;
        return;
    end

    % Potong ke wilayah step
    u_step = u(step_idx:end);
    y_step = y(step_idx:end);
    t_step = (0:length(u_step)-1)' * Ts;

    % Gain keadaan tunak
    u_ss = mean(u_step(end-min(50, length(u_step)):end));
    y_ss = mean(y_step(end-min(50, length(y_step)):end));
    if abs(u_ss) < 1e-6
        u_ss = sign(mean(u_step)) * max(abs(u_step));
    end
    K = y_ss / u_ss;  % CPS / PWM

    % Konstanta waktu: waktu naik 63.2%
    y_target = 0.632 * abs(y_ss);
    if y_target > 0
        rise_idx = find(abs(y_step) >= y_target * 0.9, 1, 'first');
        if isempty(rise_idx)
            rise_idx = length(y_step);
        end
        tau = t_step(rise_idx);
    else
        tau = 0.1;
    end

    % Waktu mati: waktu dari step sampai respons 10%
    y_10 = 0.1 * abs(y_ss);
    theta_idx = find(abs(y_step) >= y_10, 1, 'first');
    if isempty(theta_idx)
        theta_idx = 1;
    end
    theta = t_step(theta_idx);

    theta = max(theta, Ts);   % minimal 1 sampel
    tau = max(tau - theta, Ts);

    fprintf('  FOPDT: K=%.2f, tau=%.3f s, theta=%.3f s\n', K, tau, theta);

    % Tuning PID Ziegler-Nichols (klasik step-response)
    a = K * theta / tau;
    Kp = 1.2 / a;
    Ki = Kp / (2 * theta);
    Kd = Kp * theta / 2;

    % Batasi gain ke rentang wajar
    Kp = max(0.01, min(Kp, 2.0));
    Ki = max(0.0, min(Ki, 5.0));
    Kd = max(0.0, min(Kd, 0.1));

    fprintf('  Z-N PID: Kp=%.4f  Ki=%.4f  Kd=%.4f\n', Kp, Ki, Kd);

    % Plot fit step
    figure('Name', sprintf('Roda %s - Fit FOPDT', wheel_name));
    plot(t_step, y_step, 'b');
    hold on;
    t_model = t_step(1):Ts:t_step(end);
    y_model = zeros(size(t_model));
    for j = 1:length(t_model)
        if t_model(j) >= theta
            y_model(j) = K * u_ss * (1 - exp(-(t_model(j) - theta) / tau));
        end
    end
    plot(t_model, y_model, 'r--', 'LineWidth', 1.5);
    xlabel('Waktu (s)');
    ylabel('CPS');
    legend('Terukur', 'Model FOPDT');
    title(sprintf('Roda %s - fit step-response', wheel_name));
    grid on;
end
