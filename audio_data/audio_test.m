Fs = 48000;
nBits = 16;
nChannels = 1;
recTime = 5;
micID = 7;  % Default ALSA device

try
    recObj = audiorecorder(Fs, nBits, nChannels, micID);
    disp('Recording for 5 seconds...');
    recordblocking(recObj, recTime);
    disp('Recording finished.');

    y = getaudiodata(recObj);
    sound(y, Fs);

    figure;
    plot(y);
    xlabel('Samples');
    ylabel('Amplitude');
    title('Optaget signal (default mikrofon)');
catch ME
    disp(ME.message);
end
