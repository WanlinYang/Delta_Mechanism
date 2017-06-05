function client_mmc(port)

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 10 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',20); 
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
        'c: Reset gimbal encoders              d: Set desired torques to 3 JCs\n',...
        'e: Read current torques from 3 JCs    f: Set Stiffness to 3 JCs\n',...
        'g: Reset spring abs encoders          h: Read spring abs encoders\n',...
        'i: Read geometry parameter            j: Set PWM\n',...
        'k: Reset incremental encoders         l: Read incremental encoders\n',...
        'u: Set control gains                  p: Set JCs to IDLE mode\n',...
        'o: Move end-effect to an position     n: Load cubic trajectory\n',...
        'y: Set JCs to TRACK mode              r: Read actual angle data from JCs\n',...
        'q: Quit client\n']);

	
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    switch selection
        case 'a'
            input('Please move manipulator to home position.');
            fprintf(mySerial,'\n');
            input('Please load the article.');
            fprintf(mySerial,'\n');
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
            fprintf('Please enter desired torques\n');
            torque1 = input('torque1 = ');
            fprintf(mySerial, '%f\n' ,torque1);
            torque2 = input('torque2 = ');
            fprintf(mySerial, '%f\n' ,torque2);
            torque3 = input('torque3 = ');
            fprintf(mySerial, '%f\n' ,torque3);
            fprintf('Desired torques have been input\n')
        case 'e'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'f'
            stiffness = input('Please enter stiffness(Nm/rad): ');
            fprintf(mySerial,'%f\n',stiffness);
        case 'g'
            fprintf('Spring encoders have been reset');
        case 'h'
            for i=1:6
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'j'
            pwm1 = input('Enter pwm1: ');
            fprintf(mySerial, '%f\n', pwm1);
            pwm2 = input('Enter pwm2: ');
            fprintf(mySerial, '%f\n', pwm2);
            pwm3 = input('Enter pwm3: ');
            fprintf(mySerial, '%f\n', pwm3);
            fprintf('Desired PWM have been input\n')
        case 'k'
            fprintf('Incremental encoders have been reset'); 
        case 'l'
            for i=1:3
                data(i,:) = fscanf(mySerial,'%f\n');
                fprintf('%f\n',data(i,:));
            end
        case 'p'
            fprintf('JCs have been set to IDLE Mode\n'); 
        case 'i'
            R1 = input('Enter R1: ');
            fprintf(mySerial, '%f\n', R1);
            R2 = input('Enter R2: ');
            fprintf(mySerial, '%f\n', R2);
            L1 = input('Enter L1: ');
            fprintf(mySerial, '%f\n', L1);
            L2 = input('Enter L2: ');
            fprintf(mySerial, '%f\n', L2);			
		case 'u'
			Kp_c = input('Enter Kp in current controller: ');
            fprintf(mySerial, '%f\n', Kp_c);
			Ki_c = input('Enter Ki in current controller: ');
            fprintf(mySerial, '%f\n', Ki_c);
			Kp_p = input('Enter Kp in position controller: ');
            fprintf(mySerial, '%f\n', Kp_p);
			Ki_p = input('Enter Ki in position controller: ');
            fprintf(mySerial, '%f\n', Ki_p);	
			Kd_p = input('Enter Kp in position controller: ');
            fprintf(mySerial, '%f\n', Kd_p);			
        case 'o'		
            x = input('Enter x: ');
            fprintf(mySerial, '%f\n', x);
            y = input('Enter y: ');
            fprintf(mySerial, '%f\n', y);
            z = input('Enter z: ');
            fprintf(mySerial, '%f\n', z);
            fprintf('JCs have been set to HOLD Mode\n'); 
        
        case 'n'                         % example operation              
            nx = input('Enter cubic trajectory of x:'); % get the number to send
            refx = genRef(nx, 'cubic')
            ny = input('Enter cubic trajectory of y:'); % get the number to send
            refy = genRef(ny, 'cubic')
            nz = input('Enter cubic trajectory of z:'); % get the number to send
            refz = genRef(nz, 'cubic')
            if(length(refx)==length(refy) && length(refy)==length(refz))
                
                fprintf(mySerial, '%d\n',length(refx)); % send the number
                for i=1:length(refx)
                    fprintf(mySerial, '%f\n',refx(i)); % send the number
                end
                for i=1:length(refy)
                    fprintf(mySerial, '%f\n',refy(i)); % send the number
                end
                for i=1:length(refz)
                    fprintf(mySerial, '%f\n',refz(i)); % send the number
                end
            else fprintf('length of x,y,z are not equal\n'); 
            end
        case 'y'
            fprintf('JCs have been set to TRACK Mode\n'); 
        case 'r'
                nsamples = fscanf(mySerial,'%d');       % first get the number of samples being sent
                fprintf('%d\n',nsamples);
                data_temp = zeros(nsamples,6);               % two values per sample:  ref and actual
                for i=1:nsamples
                    data_temp(i,:) = fscanf(mySerial,'%f %f %f %f %f %f'); % read in data from PIC32; assume ints, in mA
                    fprintf('%f %f %f %f %f %f\n',data_temp(i,:));
                    times(i) = (i-1)*0.005;                 % 0.005 s between samples
                end
                if nsamples > 1	
                figure    
                stairs(times,data_temp(:,1:6));            % plot the reference and actual
                
                figure
                plot(data_temp(:,1))
                hold on
                plot(data_temp(:,2))
                plot(data_temp(:,3))
                plot(data_temp(:,4))
                plot(data_temp(:,5))
                plot(data_temp(:,6))
                legend('J1act','J2act','J3act','J1ref','J2ref','J3ref') 
                else
                    fprintf('Only 1 sample received\n')
                    disp(data_temp);
                end
        case 'q'
            has_quit = true;             % exit client
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
    
end
        

end