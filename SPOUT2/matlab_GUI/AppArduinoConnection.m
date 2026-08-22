%% AppArduinoConnection

% Class, similar to Ofer's ArduinoConnect, that creates an object of the
% serialport class, which calls a configureCallback function to handle
% incoming messages from a Teensy, Arduino, or other USB device. Better
% suited for DualLickGUI because of it's dynamic message handler and
% updated serialport class

classdef AppArduinoConnection < handle

    properties
        serialConnection        % Serial connection object
        messageCallbackFcn      % Callback function for receiving data
        arduinoMessageString    % Buffer for storing incoming data
    end

    methods
        % Constructor for setting up the Arduino connection
        function obj = AppArduinoConnection(app, messageHandler, baudRate)

            % notify user
            disp(['Scanning USB ports to find Arduino/Teensy/USB Serial Device...'])

            obj.messageCallbackFcn = messageHandler;  % Set the initial callback function
            obj.arduinoMessageString = '';
            arduinoPortName = obj.findFirstArduinoPort();

            if isempty(arduinoPortName)
                disp('Cannot find serial port with Arduino');
                return;
            end

            % Configure serial connection
            serialPort = serialport(arduinoPortName, baudRate); % Create a serialport object
            configureTerminator(serialPort, "LF"); % Set terminator to '\n'
            configureCallback(serialPort, "terminator", @(port, event) app.defaultHandler(port, event)); % Configure callback for incoming data
            % configureCallback(serialPort, "terminator", @(port, event) obj.messageCallbackFcn(port, event));

            obj.serialConnection = serialPort;  % Store the serial connection
        end

        % Method to change the message handler
        function setMessageHandler(obj, newHandler)
            % Update the message callback function dynamically
            obj.messageCallbackFcn = newHandler;
            configureCallback(obj.serialConnection, "terminator", @(port, event) obj.messageCallbackFcn(port, event));
        end

        % Method to send a message
        function writeString(obj, message)
            % Send a message to the Teensy
            writeline(obj.serialConnection, message);
        end

    end

    methods (Static)
        % Method to find the first Arduino/Teensy port
        function port = findFirstArduinoPort()

            % OFER
    		% finds the first port with an Arduino on it.
    		port = [];

    		% OSX code:
    		serialInfo = instrhwinfo('serial');
    		archstr = computer('arch');
    		if strcmp(archstr,'maci64')
                for portN = 1:length(serialInfo.AvailableSerialPorts)
                    portName = serialInfo.AvailableSerialPorts{portN};
                    if strfind(portName,'tty.usbmodem')
                        port = portName;
                        return
                    end
                end
            else
        		% PC code:
    		    % code from Benjamin Avants on Matlab Answers
    		    % http://www.mathworks.com/matlabcentral/answers/110249-how-can-i-identify-com-port-devices-on-windows

    		    Skey = 'HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM';
    		    % Find connected serial devices and clean up the output
    		    [~, list] = dos(['REG QUERY ' Skey]);
    		    list = strread(list,'%s','delimiter',' ');
    		    coms = 0;
    		    for i = 1:numel(list)
                    if strcmp(list{i}(1:3),'COM')
                        if ~iscell(coms)
                            coms = list(i);
                        else
                            coms{end+1} = list{i};
                        end
                    end
    		    end
    		    key = 'HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\';
    		    % Find all installed USB devices entries and clean up the output
    		    [~, vals] = dos(['REG QUERY ' key ' /s /f "FriendlyName" /t "REG_SZ"']);
    		    vals = textscan(vals,'%s','delimiter','\t');
    		    vals = cat(1,vals{:});
    		    out = 0;
    		    % Find all friendly name property entries
    		    for i = 1:numel(vals)
    		        if strcmp(vals{i}(1:min(12,end)),'FriendlyName')
    		            if ~iscell(out)
    		                out = vals(i);
    		            else
    		                out{end+1} = vals{i};
    		            end
    		        end
    		    end
    		    % Compare friendly name entries with connected ports and generate output
    		    for i = 1:numel(coms)
    		        match = strfind(out,[coms{i},')']);
    		        ind = 0;
    		        for j = 1:numel(match)
    		            if ~isempty(match{j})
    		                ind = j;
    		            end
    		        end
    		        if ind ~= 0
    		            com = str2double(coms{i}(4:end));
    		            % Trim the trailing ' (COM##)' from the friendly name - works on ports from 1 to 99
    		            if com > 9
    		                len = 8;
    		            else
    		                len = 7;
    		            end
    		            devs{i,1} = out{ind}(27:end-len);
    		            devs{i,2} = coms{i};
    		        end
    		    end
    		    % get the first arduino port
    		    for i = 1:numel(coms)
    		        [portFriendlyName, portName] = devs{i,:};
    		        if strfind(portFriendlyName, 'Arduino')
    		            port = portName;
                        disp('Connected with Arduino')
    		            return
                    elseif strfind(portFriendlyName, 'Teensy')
    		            port = portName;
                        disp('Connected with Teensy')
    		            return
                    elseif strfind(portFriendlyName, 'USB Serial Device')
    		            port = portName;
                        disp('Connected with USB Serial Device')
    		            return
                    end
                end
            end

        end
    end
end
