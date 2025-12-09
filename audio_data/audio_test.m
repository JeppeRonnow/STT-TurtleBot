
%%

Fs = 48000;
nBits = 16;
nChannels = 1;
recTime = 5;
micID1 = 7;  % Default ALSA device
micID2 = 3; %

try

    recObj1 = audiorecorder(Fs, nBits, nChannels, micID1);
    recObj2 = audiorecorder(Fs, nBits, nChannels, micID2);


    disp('Recording 5 seconds...');
    recordblocking(recObj1, recTime);
    recordblocking(recObj2, recTime);
    disp('Done recording')

    y1 = getaudiodata(recObj1);
    y2 = getaudiodata(recObj2);

    y1 = y1 / max(abs(y1));
    y2 = y2 / max(abs(y2));

    t = (0:length(y1)-1)/Fs;

    figure;
    subplot(2,1,1);
    plot(t, y1);
    title('Microphone 1');
    xlabel('Time (s)');
    ylabel('Amplitude');
    
    subplot(2,1,2);
    plot(t, y2);
    title('Microphone 2');
    xlabel('Time (s)');
    ylabel('Amplitude');
    catch ME
        disp(ME.message);
end
