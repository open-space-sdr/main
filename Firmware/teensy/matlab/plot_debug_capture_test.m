fid = fopen('debug_capture_test.hex');

rx_data = fscanf(fid,'%x ');
fclose(fid);

% Convert hex words into I/Q
rx_data_real = floor(rx_data/256);
rx_data_imag = mod(rx_data,256);
rx_data_real(rx_data_real>=128) = rx_data_real(rx_data_real>=128) - 256; % make signed
rx_data_imag(rx_data_imag>=128) = rx_data_imag(rx_data_imag>=128) - 256;
rx_data = complex(rx_data_real, rx_data_imag);

plot(real(rx_data)); hold on; plot(imag(rx_data));
