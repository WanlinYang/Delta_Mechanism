function client_mmc(port)

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 10 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',10); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;

while ~has_quit
    
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf([
        'a: Calibration                        b: Read gimbal encoders\n',...
        'c: Reset three gimbal encoders        d: send desired torques to 3 JCs\n',...
        'e: Receive current torques from 3 JCs f: Receive current torques from 3 JCs\n',...
        'g: Reset spring abs encoders          h: Read spring abs encoders\n',...
        'j: Sent PWM                           q: Quit client\n']);

    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    switch selection
        case 'a'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'b'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'c'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'd'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'e'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'f'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'g'
            fprintf('Spring encoders have been reset');
        case 'h'
            for i=1:6
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'j'
            pwm1 = input('Enter pwm1: ');
            pwm2 = input('Enter pwm2: ');
            pwm3 = input('Enter pwm3: ');
            fprintf(mySerial, '%d  %d  %d\n', [pwm1,pwm2,pwm3]);
        case 'q'
            has_quit = true;             % exit client
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
    
end
        

end