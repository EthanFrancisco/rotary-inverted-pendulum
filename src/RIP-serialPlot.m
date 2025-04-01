% For MATLAB R2019b and later
serialPort = 'COM5';  % Adjust this to the correct COM port for your system
baudRate = 9600;  % Ensure this matches the Arduino baud rate

% Create a serial object (using the newer serialport function)
s = serialport(serialPort, baudRate);
configureTerminator(s, "LF");  % This sets the end-of-line character (optional)

% Initialize data storage
angles = [];
pwmSignals = [];
timeStamps = [];

% Set up figure for plotting
figure;
subplot(3, 1, 1);
h1 = plot(0, 'b');
title('Pendulum Angle (Theta)');
xlabel('Time (ms)');
ylabel('Angle (Degrees)');
grid on;

subplot(3, 1, 2);
h2 = plot(0, 'g');
title('Motor PWM');
xlabel('Time (ms)');
ylabel('PWM Value');
grid on;

subplot(3, 1, 3);
h3 = plot(0, 'r');
title('Intervals between readings');
xlabel('Time (ms)');
ylabel('Interval (ms)');
grid on;

% Read data from serial and plot
for i = 1:1000  % Adjust the number of readings as needed
    data = readline(s);  % Read a line of data from Arduino
    
    if contains(data, 'Angle:')
        % Parse the data (e.g., "Angle: 179.5, PWM: 120")
        angleStr = extractBetween(data, 'Angle:', ',');
        pwmStr = extractBetween(data, 'PWM:', ',');
        
        angle = str2double(angleStr{1});
        pwm = str2double(pwmStr{1});
        
        % Store the data
        angles = [angles, angle];
        pwmSignals = [pwmSignals, pwm];
        timeStamps = [timeStamps, toc*1000];  % Time in milliseconds
        
        % Update the plots
        set(h1, 'YData', angles, 'XData', timeStamps);
        set(h2, 'YData', pwmSignals, 'XData', timeStamps);
        
        % Update intervals plot
        if length(timeStamps) > 1
            intervals = diff(timeStamps);
            set(h3, 'YData', intervals, 'XData', timeStamps(2:end));
        end
        drawnow;
    end
end

% Close serial port
clear s;
