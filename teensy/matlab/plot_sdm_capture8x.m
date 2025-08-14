%fid = fopen('rx.hex');
fid = fopen('sdm_capture.hex');
rx_data = fscanf(fid,'%x ');
fclose(fid);
rx = zeros(length(rx_data)*8,1);
for k=1:length(rx_data)
    b = 2*(dec2bin(rx_data(k),16) == '1')-1;
    
    rx(8*k-7:8*k) = fliplr(b(1:8)).' + 1i*fliplr(b(9:16)).';
end


figure(1); title('Sigma-delta frequency domain');
grid on;
hold on;
plot(linspace(0,640e6, length(rx))/1e6, - 92 + 20*log10(conv(abs(fft(real(rx))), ones(10,1)/10,'same')));
plot(linspace(0,640e6, length(rx))/1e6, - 92 + 20*log10(conv(abs(fft(imag(rx))), ones(10,1)/10,'same')));
plot(linspace(0,640e6, length(rx))/1e6, - 92 + 20*log10(conv(abs(fft(rx)), ones(10,1)/10,'same')),'k');
legend('I', 'Q', 'IQ')

h_cic = [-1	-1	-2	-2	-3	-3	-4	-4	-3	-2	-1	0	2	4	6	8	10	12	14	15	17 18	19	19	19	19	18	17	15	14	12	10	8	6	4	2	0	-1	-2	-3	-4	-4	-3	-3	-2	-2	-1	-1];

rx_filtered = conv(rx,h_cic,'same');
rx_filtered = rx_filtered(1:8:end);

%plot(linspace(0,640e6 / 8, length(rx_filtered))/1e6, - 92 + 20*log10(conv(abs(fft(rx_filtered)), ones(10,1)/10,'same')),'k');

xlabel('MHz');
ylabel('PSD dB')

figure(2); hold on; title('Filtered sigma delta (time domain)');

plot(real(rx_filtered)); hold on;plot(imag(rx_filtered));
legend('I', 'Q', 'IQ');


% CIC Filter & QPSK Matched filter
b_rnd = [-1	-1	-2	-2	-3	-3	-4	-4	-3	-2	-1	0	2	4	6	8	10	12	14	15	17 18	19	19	19	19	18	17	15	14	12	10	8	6	4	2	0	-1	-2	-3	-4	-4	-3	-3	-2	-2	-1	-1];
rx2 = conv(rx, b_rnd);
rx2 = rx2(1:8:end);
plot(linspace(0,80e6, length(rx2))/1e6, - 92 + 20*log10(conv(abs(fft((rx2))), ones(10,1)/10,'same')));
% Now filter with QPSK pulse filter to remove more shaping
SRRC_Len = 31;
SRRC_Beta = 0.5;
filter_tap_times = [(16+1 - 7/8*(16+1)):(7/8):(16+1-7/8), ((16+1):(7/8):((16+1)+(7.0/8.0)*(16+1)))];
srrc = rcosdesign(SRRC_Beta,ceil(SRRC_Len/2),2,'sqrt');  % 2 samples or taps per symbol
srrc = interp1(1:length(srrc), srrc, filter_tap_times, 'spline'); % resample at specified tap times
srrc_filter = round(255*7/8*srrc); % Quantize
rx3 = conv(rx2, srrc_filter, 'same') / 256;
figure(3);
plot(linspace(0,80e6, length(rx3(1:end)))/1e6, - 92 + 20*log10(conv(abs(fft((rx3(1:end)))), ones(10,1)/10,'same')));
title('Resampled to 80 MSPS');