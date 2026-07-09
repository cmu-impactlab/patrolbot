               ; rotation.
LaserTh 0.00000          ; range [-180, 180],  Rotation (in deg) of the laser
                         ; (+ counterclockwise, - clockwise).
LaserZ 0                 ; minimum 0,  Height (in mm) of the laser from the
                         ; ground. 0 means unknown.
LaserIgnore              ; Angles (in deg) at which to ignore readings, +/1 one
                         ; degree. Angles are entered as strings, separated by
                         ; a space.
LaserFlipped true        ; Laser_2 is upside-down.
LaserType urg            ; Type of laser.
LaserPortType serial     ; Type of port the laser is on.
LaserPort COM5           ; Port the laser is on.
LaserPowerOutput         ; Power output that controls this laser's power.
LaserStartingBaudChoice  ; StartingBaud for this laser. Leave blank to use the
                         ; default.
LaserAutoBaudChoice      ; AutoBaud for this laser. Leave blank to use the
                         ; default.
LaserPowerControlled true ; When enabled (true), this indicates that the power
                         ; to the laser is controlled by the serial port line.
LaserMaxRange 0          ; Maximum range (in mm) to use for the laser. This
                         ; should be specified only when the range needs to be
                         ; shortened. 0 to use the default range.
LaserCumulativeBufferSize 0 ; Cumulative buffer size to use for the laser. 0 to
                         ; use the default.
LaserStartDegrees -55    ; Start angle (in deg) for the laser. This may be used
                         ; to constrain the angle. Fractional degrees are
                         ; permitted. Leave blank to use the default.
LaserEndDegrees 55       ; End angle (in deg) for the laser. This may be used
                         ; to constrain the angle. Fractional degreees are
                         ; permitted. Leave blank to use the default.
LaserDegreesChoice       ; Degrees choice for the laser. This may be used to
                         ; constrain the range. Leave blank to use the default.
LaserIncrement 1.0       ; Increment (in deg) for the laser. Fractional degrees
                         ; are permitted. Leave blank to use the default.
LaserIncrementChoice     ; Increment choice for the laser. This may be used to
                         ; increase the increment. Leave blank to use the
                         ; default.
LaserUnitsChoice         ; Units for the laser. This may be used to increase
                         ; the size of the units. Leave blank to use the
                         ; default.
LaserReflectorBitsChoice  ; ReflectorBits for the laser. Leave blank to use the
                         ; default.

Section Battery_1
;  Information about the connection to this battery.
;SectionFlags for Battery_1: 
BatteryAutoConnect false ; Battery_1 exists and should be automatically
                         ; connected at startup.
BatteryType              ; Type of battery.
BatteryPortType          ; Port type that the battery is on.
BatteryPort              ; Port the battery is on.
BatteryBaud 0            ; Baud rate to use for battery communication (9600,
                         ; 19200, 38400, etc.).

Section LCD_1
;  The physical definition of this LCD.
;SectionFlags for LCD_1: 
LCDAutoConnect false     ; LCD_1 exists and should automatically be connected
                         ; at startup.
LCDDisconnectOnConnectFailure false ; The LCD is a key component and is
                         ; required for operation. If this is enabled and there
                         ; is a failure in the LCD communications, then the
                         ; robot will restart.
LCDType                  ; Type of LCD.
LCDPortType              ; Port type that the LCD is on.
LCDPort                  ; Port that the LCD is on.
LCDPowerOutput           ; Power output that controls this LCD's power.
LCDBaud 0                ; Baud rate for the LCD communication (9600, 19200,
                         ; 38400, etc.).

Section PTZ 1 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 1 parameters: 
PTZAutoConnect true      ; If true, connect to this PTZ by default.
PTZType vcc4             ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section PTZ 2 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 2 parameters: 
PTZAutoConnect false     ; If true, connect to this PTZ by default.
PTZType unknown          ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section PTZ 3 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 3 parameters: 
PTZAutoConnect false     ; If true, connect to this PTZ by default.
PTZType unknown          ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section PTZ 4 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 4 parameters: 
PTZAutoConnect false     ; If true, connect to this PTZ by default.
PTZType unknown          ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section PTZ 5 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 5 parameters: 
PTZAutoConnect false     ; If true, connect to this PTZ by default.
PTZType unknown          ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section PTZ 6 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 6 parameters: 
PTZAutoConnect false     ; If true, connect to this PTZ by default.
PTZType unknown          ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section PTZ 7 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 7 parameters: 
PTZAutoConnect false     ; If true, connect to this PTZ by default.
PTZType unknown          ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section PTZ 8 parameters
;  Information about the connection to a pan/tilt unit (PTU) or pan/tilt/zoom
;  control (PTZ) of a camera
;SectionFlags for PTZ 8 parameters: 
PTZAutoConnect false     ; If true, connect to this PTZ by default.
PTZType unknown          ; PTZ or PTU type
PTZInverted false        ; If unit is mounted inverted (upside-down)
PTZSerialPort none       ; serial port, or none if not using serial port
                         ; communication
PTZRobotAuxSerialPort -1 ; Pioneer aux. serial port, or -1 if not using aux.
                         ; serial port for communication.
PTZAddress none          ; IP address or hostname, or none if not using network
                         ; communication.
PTZTCPPort 80            ; TCP Port to use for HTTP network connection
PTZUsername              ; Username, if required by camera
PTZPassword              ; Password, if required by camera

Section Video 1 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 1 parameters: 
VideoAutoConnect true    ; If true, connect to this device by default.
VideoType v4l            ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera

Section Video 2 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 2 parameters: 
VideoAutoConnect false   ; If true, connect to this device by default.
VideoType unknown        ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera

Section Video 3 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 3 parameters: 
VideoAutoConnect false   ; If true, connect to this device by default.
VideoType unknown        ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera

Section Video 4 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 4 parameters: 
VideoAutoConnect false   ; If true, connect to this device by default.
VideoType unknown        ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera

Section Video 5 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 5 parameters: 
VideoAutoConnect false   ; If true, connect to this device by default.
VideoType unknown        ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera

Section Video 6 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 6 parameters: 
VideoAutoConnect false   ; If true, connect to this device by default.
VideoType unknown        ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera

Section Video 7 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 7 parameters: 
VideoAutoConnect false   ; If true, connect to this device by default.
VideoType unknown        ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera

Section Video 8 parameters
;  Information about the connection to a video acquisition device,
;  framegrabber, or camera
;SectionFlags for Video 8 parameters: 
VideoAutoConnect false   ; If true, connect to this device by default.
VideoType unknown        ; Device type
VideoInverted false      ; If image should be flipped (for cameras mounted
                         ; inverted/upside-down)
VideoWidth -1            ; Desired width of image, or -1 for default
VideoHeight -1           ; Desired height of image, or -1 for default
VideoDeviceIndex -1      ; Device index, or -1 for default
VideoDeviceName none     ; Device name (overrides VideoDeviceIndex)
VideoChannel -1          ; Input channel, or -1 for default
VideoAnalogSignalFormat  ; NTSC or PAL, or empty for default. Only used for
                         ; analog framegrabbers.
VideoAddress none        ; IP address or hostname, or none if not using network
                         ; communication.
VideoTCPPort 80          ; TCP Port to use for HTTP network connection
VideoUsername            ; Username, if required by camera
VideoPassword            ; Password, if required by camera
