clc;
clear;
%% Parameters for filter
fs = 16000; % Samplerate in Hz
Ts = 1/fs;  % Timesample

low_cut_hz = 80;       % Lower cutoff freq in Hz
high_cut_hz = 5000;     % Upper cutoff freq in Hz

order = 8;  % Desired order for filter

%% Normalize frequencies
nquist = 0.5 * fs;

Wp = [low_cut_hz, high_cut_hz] / nquist;    % Normalized cutoff frequencies

%% Build filters
[n_build, d_build] = butter(order, Wp, 'bandpass'); % Create butterworth bandpass filter

%% Filter created by code
n_code = [0.1762395644802633 0 -0.7049582579210532 0 1.05743738688158 0 -0.7049582579210532 0 0.1762395644802633];
d_code = [1 -2.925539040072373 2.733933428394804 -1.086408898679309 0.9673192257157804 -0.8480825196357736 0.05717299406645649 0.0687103301887228 0.03290930798725278];d = [1 -2.925539040072373 2.733933428394804 -1.086408898679309 0.9673192257157804 -0.8480825196357736 0.05717299406645649 0.0687103301887228 0.03290930798725278];

%% Plot filter build in matlab with filter from code
figure;

% Matlab build filter
[h, f] = freqz(n_build, d_build, 4096, fs);
subplot(2, 1, 1);
semilogx(f, 20*log10(abs(h)));
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('MatLab build filter');
xlim([-100 10000]);
ylim([-200, 100]);

% Code build filter
[h, f] = freqz(n_code, d_code, 4096, fs);
subplot(2, 1, 2);
semilogx(f, 20*log10(abs(h)));
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Code build filter');
xlim([-100 10000]);
ylim([-200, 100]);

%% Load audio file
[audio, fs_audio] = audioread('Raw.wav');

% Handle stero audio
if size(audio, 2) > 1
    audio = mean(audio, 2);
end

t = (0:length(audio)-1 / fs_audio);

%% Filter audio
% Filter with respect for phase shift
audio_filter = filtfilt(n_build, d_build, audio);

% Normalize
audio_norm = audio_filter / max(abs(audio_filter));

% Filter normalized
audio_norm_filter = filtfilt(n_build, d_build, audio/max(abs(audio)));

%% Display audio signals
figure('Name', order+"th order filter", 'NumberTitle','off');

% Raw audio
subplot(6,1,1);
plot(t, audio);
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Raw audio signal');
xlim([0,length(audio)]);
ylim([-1,1]);

% Filtered audio
subplot(6,1,2);
plot(t, audio_filter);
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Filtered audio signal');
xlim([0,length(audio_filter)]);
ylim([-1,1]);

% Filter audio normalized
subplot(6,1,3);
plot(t, audio_norm);
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Normalized filtered audio');
xlim([0, length(audio_norm)]);
ylim([-1,1]);

% Normalize and filter audio
subplot(6,1,4);
plot(t, audio_norm_filter);
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Filtered normalized audio');
xlim([0, length(audio_norm_filter)]);
ylim([-1,1]);

% Filter audio on top
subplot(6,1,5);
plot(t, audio, 'b');
hold on;
plot(t, audio_filter, 'r');
xlabel('Time (seconds');
ylabel('Amplitude');
title('Filtered on top of raw');
xlim([0, length(audio_filter)]);
ylim([-1, 1]);
hold off;

% Plot audio diffrence
subplot(6,1,6);
plot(t, audio, 'b');
hold on;
plot(t, audio-audio_filter);
xlabel('Time (seconds');
ylabel('Amplitude difference');
title('Audio difference');
xlim([0, length(audio_filter)]);
hold off;

%% Playback audio

% Playback raw audio
disp('Playing original sound');
sound(audio, fs_audio);
pause(length(audio)/fs_audio + 1);

% Playback filtered audio
disp('Playing filtered sound');
sound(audio_filter, fs_audio);
pause(length(audio_filter)/fs_audio + 1);

% Playback normalized filtered audio
disp('Playing normalized filtered audio');
sound(audio_norm, fs_audio);
pause(length(audio_norm)/fs_audio + 1);

% Playback filtered normalized audio
disp('Playing filtered normalized audio');
sound(audio_norm_filter, fs_audio);