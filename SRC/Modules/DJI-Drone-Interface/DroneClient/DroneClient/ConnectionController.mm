//
//  ConnectionController.m
//  DroneClient
//
//  Created by Ben Choi on 2/24/21.
//

#import "ConnectionController.h"
#import "DJIUtils.h"
#import "Constants.h"
#import "DroneComms.hpp"
#import "VideoPreviewerSDKAdapter.h"
#import "ImageUtils.h"
#import "Image.hpp"
#import "Drone.hpp"

@interface ConnectionController ()<DJISDKManagerDelegate, DJICameraDelegate, DJIBatteryDelegate, DJIBatteryAggregationDelegate, DJIFlightControllerDelegate, NSStreamDelegate, DJIVideoFeedListener, VideoFrameProcessor>

@end

@implementation ConnectionController

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    [self registerApp];
    [self configureConnectionToProduct];
}

- (void)viewWillDisappear:(BOOL)animated
{
    [super viewWillDisappear:animated];
    [[DJIVideoPreviewer instance] unSetView];
           
    if (self.previewerAdapter) {
        [self.previewerAdapter stop];
        self.previewerAdapter = nil;
    }
}

- (void)viewDidLoad {
    [super viewDidLoad];
    [self connectToServer];
}

#pragma mark TCP Connection

- (void) sendPacket:(DroneInterface::Packet *)packet {
    NSData *data = [[NSData alloc] initWithBytesNoCopy:packet->m_data.data() length:packet->m_data.size() freeWhenDone:false];
    const unsigned char *bytes= (const unsigned char *)(data.bytes);
    
    unsigned int bytes_written = 0;
    while (bytes_written != packet->m_data.size()) {
        int remaining = data.length - bytes_written;
        const unsigned char *bytesNew = bytes + bytes_written;
        bytes_written += [outputStream write:bytesNew maxLength:remaining];
        
        [NSThread sleepForTimeInterval: 0.001];
    }
}

- (void) sendPacket_CoreTelemetry {
    DroneInterface::Packet_CoreTelemetry packet_core;
    DroneInterface::Packet packet;
    
    packet_core.IsFlying = self->_isFlying;
    packet_core.Latitude = self->_latitude;
    packet_core.Longitude = self->_longitude;
    packet_core.Altitude = self->_altitude;
    packet_core.HAG = self->_HAG;
    packet_core.V_N = self->_velocity_n;
    packet_core.V_N = self->_velocity_e;
    packet_core.V_D = self->_velocity_d;
    packet_core.Yaw = self->_yaw;
    packet_core.Pitch = self->_pitch;
    packet_core.Roll = self->_roll;
    
    packet_core.Serialize(packet);
    
    [self sendPacket:&packet];
}

- (void) sendPacket_ExtendedTelemetry {
    DroneInterface::Packet_ExtendedTelemetry packet_extended;
    DroneInterface::Packet packet;
    
    packet_extended.GNSSSatCount = self->_GNSSSatCount;
    packet_extended.GNSSSignal = self->_GNSSSignal;
    packet_extended.MaxHeight =self->_max_height;
    packet_extended.MaxDist = self->_max_dist;
    packet_extended.BatLevel = self->_bat_level;
    packet_extended.BatWarning = self->_bat_warning;
    packet_extended.WindLevel = self->_wind_level;
    packet_extended.DJICam = self->_dji_cam;
    packet_extended.FlightMode = self->_flight_mode;
    packet_extended.MissionID = self->_mission_id;
    packet_extended.DroneSerial = std::string([self->_drone_serial UTF8String]);
    
    packet_extended.Serialize(packet);
    
    [self sendPacket:&packet];
}

- (void) sendPacket_Image {
    DroneInterface::Packet_Image packet_image;
    DroneInterface::Packet packet;
    
//    Uncomment below to show frame being sent in packet.
//    [self showCurrentFrameImage];
    
    CVPixelBufferRef pixelBuffer;
    if (self->_currentPixelBuffer) {
        pixelBuffer = self->_currentPixelBuffer;
        UIImage* image = [self imageFromPixelBuffer:pixelBuffer];
        packet_image.TargetFPS = [DJIVideoPreviewer instance].currentStreamInfo.frameRate;
        unsigned char *bitmap = [ImageUtils convertUIImageToBitmapRGBA8:image];
        packet_image.Frame = new Image(bitmap, image.size.height, image.size.width, 4);
    }
    
    packet_image.Serialize(packet);
    
    [self sendPacket:&packet];
}

- (void) sendPacket_Acknowledgment:(BOOL) positive withPID:(UInt8)source_pid {
    DroneInterface::Packet_Acknowledgment packet_acknowledgment;
    DroneInterface::Packet packet;
    
    packet_acknowledgment.Positive = positive ? 1 : 0;
    packet_acknowledgment.SourcePID = source_pid;
    
    packet_acknowledgment.Serialize(packet);
    
    [self sendPacket:&packet];
}

- (void) sendPacket_MessageString:(NSString*)msg ofType:(UInt8)type {
    
    DroneInterface::Packet_MessageString packet_msg;
    DroneInterface::Packet packet;
    
    packet_msg.Type = type;
    packet_msg.Message = std::string([msg UTF8String]);
    
    packet_msg.Serialize(packet);
    
    [self sendPacket: &packet];
}

// Executes when SEND DEBUG COMMAND button is pressed
- (IBAction)sendDebugMessage:(id)sender {
//    [self sendPacket_CoreTelemetry];
//    [self sendPacket_ExtendedTelemetry];
//    [self sendPacket_Image];
//    [self sendPacket_Acknowledgment:YES withPID:4];
    [self sendPacket_MessageString:TEST_MESSAGE ofType: 2];
}

- (void) dataReceivedHandler:(uint8_t *)buffer bufferSize: (uint32_t) size withPacket: (DroneInterface::Packet*) packet_fragment {
    
    unsigned int i = 0;
    while(!packet_fragment->IsFinished() && i < size) {
        packet_fragment->m_data.push_back(buffer[i++]);
    }

    if (packet_fragment->IsFinished()) {
        uint8_t PID;
        packet_fragment->GetPID(PID);
        switch(PID) {
            case 255U: {
                DroneInterface::Packet_EmergencyCommand* packet_ec = new DroneInterface::Packet_EmergencyCommand();
                if (packet_ec->Deserialize(*packet_fragment)) {
                    NSLog(@"Successfully deserialized Emergency Command packet.");
                    [self sendPacket_Acknowledgment:1 withPID:PID];
                    
                    [self stopDJIWaypointMission];
                    
                    if (packet_ec->Action == 1) {
                        [[DJIUtils fetchFlightController] startLandingWithCompletion:^(NSError * _Nullable error) {
                        
                        }];
                    } else if (packet_ec->Action == 2) {
                        [[DJIUtils fetchFlightController] startGoHomeWithCompletion:^(NSError * _Nullable error) {
                        
                        }];
                    }
                } else {
                    NSLog(@"Error: Tried to deserialize invalid Emergency Command packet.");
                    [self sendPacket_Acknowledgment:0 withPID:PID];
                }
                break;
            }
            case 254U: {
                DroneInterface::Packet_CameraControl* packet_cc = new DroneInterface::Packet_CameraControl();
                if (packet_cc->Deserialize(*packet_fragment)) {
                    NSLog(@"Successfully deserialized Camera Control packet.");
                    [self sendPacket_Acknowledgment:1 withPID:PID];
                    
                    if (packet_cc->Action == 0) { // stop live feed
                        self->_dji_cam = 1;
                    } else if (packet_cc->Action == 1) { // start live feed
                        self->_dji_cam = 2;
                        self->_target_fps = packet_cc->TargetFPS;
                    }
                } else {
                    NSLog(@"Error: Tried to deserialize invalid Camera Control packet.");
                    [self sendPacket_Acknowledgment:0 withPID:PID];
                }
                break;
            }
            case 253U: {
                DroneInterface::Packet_ExecuteWaypointMission* packet_ewm = new DroneInterface::Packet_ExecuteWaypointMission();
                if (packet_ewm->Deserialize(*packet_fragment)) {
                    NSLog(@"Successfully deserialized Execute Waypoint Mission packet.");
                    [self sendPacket_Acknowledgment:1 withPID:PID];
                    
                    struct DroneInterface::WaypointMission* mission;
                    mission->LandAtLastWaypoint = packet_ewm->LandAtEnd;
                    mission->CurvedTrajectory = packet_ewm->CurvedFlight;
                    mission->Waypoints = packet_ewm->Waypoints;
                    
                    [self executeDJIWaypointMission:mission];
                } else {
                    NSLog(@"Error: Tried to deserialize invalid Execute Waypoint Mission packet.");
                    [self sendPacket_Acknowledgment:0 withPID:PID];
                }
                break;
            }
            case 252U: {
                DroneInterface::Packet_VirtualStickCommand* packet_vsc = new DroneInterface::Packet_VirtualStickCommand();
                if (packet_vsc->Deserialize(*packet_fragment)) {
                    NSLog(@"Successfully deserialized Virtual Stick Command packet.");
                    [self sendPacket_Acknowledgment:1 withPID:PID];
                    
                    [self stopDJIWaypointMission];
                    
                    if (packet_vsc->Mode == 0) {
                        struct DroneInterface::VirtualStickCommand_ModeA* command;
                        command->Yaw = packet_vsc->Yaw;
                        command->V_North = packet_vsc->V_x;
                        command->V_East = packet_vsc->V_y;
                        command->HAG = packet_vsc->HAG;
                        command->timeout = packet_vsc->timeout;
                        [self executeVirtualStickCommand_ModeA:command];
                    } else if (packet_vsc->Mode == 1) {
                        struct DroneInterface::VirtualStickCommand_ModeB* command;
                        command->Yaw = packet_vsc->Yaw;
                        command->V_Forward = packet_vsc->V_x;
                        command->V_Right = packet_vsc->V_y;
                        command->HAG = packet_vsc->HAG;
                        command->timeout = packet_vsc->timeout;
                        [self executeVirtualStickCommand_ModeB:command];
                    }
                    
                    self->_time_of_last_virtual_stick_command = [NSDate date];
                } else {
                    NSLog(@"Error: Tried to deserialize invalid Virtual Stick Command packet.");
                    [self sendPacket_Acknowledgment:0 withPID:PID];
                }
                break;
            }
        }
    }
}

- (void)stream:(NSStream *)theStream handleEvent:(NSStreamEvent)streamEvent {

    NSLog(@"stream event %lu", streamEvent);

    switch (streamEvent) {

        case NSStreamEventOpenCompleted: {
            NSLog(@"Stream opened");
            _serverConnectionStatusLabel.text = @"Server Status: Connected";
            break;
        }
        case NSStreamEventHasBytesAvailable:
            _serverConnectionStatusLabel.text = @"Server Status: Reading from server";
            if (theStream == inputStream)
            {
                uint8_t buffer[1024];
                NSInteger len;

                DroneInterface::Packet* packet_fragment = new DroneInterface::Packet();
                while ([inputStream hasBytesAvailable])
                {
                    len = [inputStream read:buffer maxLength:sizeof(buffer)];
                    if (len > 0)
                    {
                        _serverConnectionStatusLabel.text = @"Server Status: Connected";
                        [self dataReceivedHandler:buffer bufferSize:1024 withPacket:packet_fragment];
                    }
                }
            }
            break;

        case NSStreamEventHasSpaceAvailable:
            NSLog(@"Stream has space available now");
            break;

        case NSStreamEventErrorOccurred:
             NSLog(@"%@",[theStream streamError].localizedDescription);
            break;

        case NSStreamEventEndEncountered:

            [theStream close];
            [theStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
            _serverConnectionStatusLabel.text = @"Server Status: Not Connected";
            NSLog(@"close stream");
            break;
        default:
            NSLog(@"Unknown event");
    }

}

- (void)connectToServer {
    _serverConnectionStatusLabel.text = @"Server Status: Connecting...";
    NSLog(@"Setting up connection to %@ : %i", ipAddress, [port intValue]);
    CFStreamCreatePairWithSocketToHost(kCFAllocatorDefault, (__bridge CFStringRef) ipAddress, [port intValue], &readStream, &writeStream);

    messages = [[NSMutableArray alloc] init];

    [self open];
}

- (void)disconnect {
    _serverConnectionStatusLabel.text = @"Server Status: Disconnecting...";
    [self close];
}

- (void)open {

    NSLog(@"Opening streams.");

    outputStream = (__bridge NSOutputStream *)writeStream;
    inputStream = (__bridge NSInputStream *)readStream;

    [outputStream setDelegate:self];
    [inputStream setDelegate:self];

    [outputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [inputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];

    [outputStream open];
    [inputStream open];
}

- (void)close {
    NSLog(@"Closing streams.");
    [inputStream close];
    [outputStream close];
    [inputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [outputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [inputStream setDelegate:nil];
    [outputStream setDelegate:nil];
    inputStream = nil;
    outputStream = nil;
}

#pragma mark DJI Methods

- (void) configureConnectionToProduct {
    _uavConnectionStatusLabel.text = @"UAV Status: Connecting...";
#if ENABLE_DEBUG_MODE
    [DJISDKManager enableBridgeModeWithBridgeAppIP:@"192.168.1.56"];
#else
    [DJISDKManager startConnectionToProduct];
#endif

    [[DJIVideoPreviewer instance] start];
    self.previewerAdapter = [VideoPreviewerSDKAdapter adapterWithDefaultSettings];
    [self.previewerAdapter start];
    [[DJIVideoPreviewer instance] registFrameProcessor:self];
    [[DJIVideoPreviewer instance] setEnableHardwareDecode:true];
    self->_frame_count = 0;
    self->_dji_cam = 2;
    self->_target_fps = 30;
    self->_time_of_last_virtual_stick_command = [NSDate date];
}

- (void) videoProcessFrame:(VideoFrameYUV *)frame {
    if (frame->cv_pixelbuffer_fastupload != nil) {
        if (self->_dji_cam == 2 && (self->_frame_count % ((int) self->_target_fps) == 0)) {
            CVPixelBufferRef pixelBuffer = (CVPixelBufferRef) frame->cv_pixelbuffer_fastupload;
            if (self->_currentPixelBuffer) {
                CVPixelBufferRelease(self->_currentPixelBuffer);
            }
            self->_currentPixelBuffer = pixelBuffer;
            CVPixelBufferRetain(pixelBuffer);
            
            self->_frame_count = 1;
            
//            [self sendPacket_MessageString:@"VIDEO FRAME WOULD HAVE SENT NOW" ofType:1]; // for checking frame timing
            [self sendPacket_Image];
        }
        self->_frame_count++;
    } else {
        self->_currentPixelBuffer = nil;
    }
}

- (BOOL)videoProcessorEnabled {
    return YES;
}

- (UIImage *)imageFromPixelBuffer:(CVPixelBufferRef)pixelBufferRef {
    CVImageBufferRef imageBuffer =  pixelBufferRef;
    CIImage* sourceImage = [[CIImage alloc] initWithCVPixelBuffer:imageBuffer options:nil];
    CGSize size = sourceImage.extent.size;
    UIGraphicsBeginImageContext(size);
    CGRect rect;
    rect.origin = CGPointZero;
    rect.size = size;
    UIImage *remImage = [UIImage imageWithCIImage:sourceImage];
    [remImage drawInRect:rect];
    UIImage *result = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
    return result;
}

- (void)showCurrentFrameImage {
    CVPixelBufferRef pixelBuffer;
    if (self->_currentPixelBuffer) {
        pixelBuffer = self->_currentPixelBuffer;
        UIImage* image = [self imageFromPixelBuffer:pixelBuffer];
        if (image) {
            UIImageView* imgView = [[UIImageView alloc] initWithFrame:CGRectMake(0, 0, image.size.width / 4, image.size.height / 4)];
            imgView.image = image;
            [self.fpvPreviewView addSubview:imgView];
        }
    }
}

#pragma mark DJISDKManagerDelegate Method

- (void)productConnected:(DJIBaseProduct *)product
{
    if (product){
        _uavConnectionStatusLabel.text = @"UAV Status: Connected";
        
        DJIFlightController* flightController = [DJIUtils fetchFlightController];
        if (flightController) {
            flightController.delegate = self;
        }
        
        DJICamera *camera = [DJIUtils fetchCamera];
        if (camera != nil) {
            camera.delegate = self;
        }
        
        DJIBattery *battery = [DJIUtils fetchBattery];
        if (battery != nil) {
            battery.delegate = self;
        }
        
        [flightController getSerialNumberWithCompletion:^(NSString * serialNumber, NSError * error) {
            self->_drone_serial = serialNumber;
        }];
        
        [flightController setVirtualStickModeEnabled:TRUE
                                      withCompletion:^(NSError * _Nullable error) {
        
        }];
        
        [[DJIUtils fetchFlightController] setVerticalControlMode:DJIVirtualStickVerticalControlModePosition];
        [[DJIUtils fetchFlightController] setRollPitchControlMode:DJIVirtualStickRollPitchControlModeVelocity];
        [[DJIUtils fetchFlightController] setYawControlMode:DJIVirtualStickYawControlModeAngle];
        
    }
    
    [self setExtendedTelemetryKeyedParameters];
}

- (void)productDisconnected
{
    _uavConnectionStatusLabel.text = @"UAV Status: Not Connected";
}

- (void)registerApp
{
   [DJISDKManager registerAppWithDelegate:self];
}

- (void)appRegisteredWithError:(NSError *)error
{
    NSString* message;
    if (error) {
        message = @"Register App Failed! Please enter your App Key in the plist file and check the network.";
        _registrationStatusLabel.text = @"Registration Status: FAILED";
        
    } else {
        message = @"App successfully registered";
        _registrationStatusLabel.text = @"Registration Status: Registered";
    }
    NSLog(@"%@", message);
}

#pragma mark - DJICameraDelegate

-(void) camera:(DJICamera*)camera didUpdateSystemState:(DJICameraSystemState*)systemState
{
    if (systemState.mode == DJICameraModePlayback ||
        systemState.mode == DJICameraModeMediaDownload) {
        if (self->needToSetMode) {
            self->needToSetMode = NO;
            [camera setMode:DJICameraModeShootPhoto withCompletion:^(NSError * _Nullable error) {
            }];
        }
    }
}

- (void)camera:(DJICamera *_Nonnull)camera
    didReceiveVideoData:(nonnull uint8_t *)videoBuffer
                 length:(size_t)size
{
    
}

#pragma mark - DJIBatteryDelegate

// Use keyed parameters method for battery level because DJIBatteryDelegate unresponsive for some reason. See https://github.com/dji-sdk/Mobile-SDK-iOS/blob/master/docs/README-KeyedInterface.md for more info.
- (void)setExtendedTelemetryKeyedParameters {
    DJIKey * batteryOneKey = [DJIBatteryKey keyWithIndex:0 andParam:DJIBatteryParamChargeRemainingInPercent];
    DJIKey * batteryTwoKey = [DJIBatteryKey keyWithIndex:1 andParam:DJIBatteryParamChargeRemainingInPercent];
    [[DJISDKManager keyManager] startListeningForChangesOnKey: batteryOneKey
                                                 withListener: self
                                               andUpdateBlock: ^(DJIKeyedValue * _Nullable oldKeyedValue, DJIKeyedValue * _Nullable newKeyedValue) {
                                                if (newKeyedValue) {
                                                    self->_bat_level_one = [newKeyedValue.value intValue];
                                                    self->_bat_level = (self->_bat_level_one + self->_bat_level_two) / 2;
                                                }
                                            }];
    [[DJISDKManager keyManager] startListeningForChangesOnKey: batteryTwoKey
                                                 withListener: self
                                               andUpdateBlock: ^(DJIKeyedValue * _Nullable oldKeyedValue, DJIKeyedValue * _Nullable newKeyedValue) {
                                                if (newKeyedValue) {
                                                    self->_bat_level_two = [newKeyedValue.value intValue];
                                                    self->_bat_level = (self->_bat_level_one + self->_bat_level_two) / 2;
                                                }
                                            }];
}

#pragma mark - DJIFlightControllerDelegate

- (void)flightController:(DJIFlightController *)fc didUpdateState:(DJIFlightControllerState *)state
{
    self->_GNSSSignal = [DJIUtils getGNSSSignal:[state GPSSignalLevel]];
    if([DJIUtils gpsStatusIsGood:[state GPSSignalLevel]])
    {
        self->_latitude = state.aircraftLocation.coordinate.latitude;
        self->_longitude = state.aircraftLocation.coordinate.longitude;
        self->_HAG = state.aircraftLocation.altitude;
        self->_altitude = state.takeoffLocationAltitude + self->_HAG;
        
    }
    
    self->_isFlying = state.isFlying ? 1 : 0;
    self->_velocity_n = state.velocityX;
    self->_velocity_e = state.velocityY;
    self->_velocity_d = state.velocityZ;
    self->_yaw = state.attitude.yaw;
    self->_pitch = state.attitude.pitch;
    self->_roll = state.attitude.roll;
    
    self->_GNSSSatCount = state.satelliteCount;
    self->_max_height = state.hasReachedMaxFlightHeight ? 1 : 0;
    self->_max_dist = state.hasReachedMaxFlightRadius ? 1 : 0;
    if (state.isLowerThanSeriousBatteryWarningThreshold) {
        self->_bat_warning = 2;
    } else {
        if (state.isLowerThanBatteryWarningThreshold) {
            self->_bat_warning = 1;
        } else {
            self->_bat_warning = 0;
        }
    }
    self->_wind_level = [DJIUtils getWindLevel:[state windWarning]];
    self->_flight_mode = [DJIUtils getFlightMode:[state flightMode]];

//  KNOWN BUG: Behavior leading to _dji_cam = 0 is currently undefined.
//    if (!self->_camera.isConnected) {
//        self->_dji_cam = 0;
//    } else {
//        if (self->_dji_cam == 0) {
//            self ->_dji_cam = 2;
//        }
//    }
    self->_mission_id = 0;
    
    double time_since_last_virtual_stick_command = [self->_time_of_last_virtual_stick_command timeIntervalSinceNow];
    if (time_since_last_virtual_stick_command > self->_virtual_stick_command_timeout) {
        struct DroneInterface::VirtualStickCommand_ModeA* command;
        command->V_North = 0;
        command->V_East = 0;
        [self executeVirtualStickCommand_ModeA:command];
    }
    
//  KNOWN BUG: Different delay values or slow connections may lead to errors in server-side deserialization
    [self sendPacket_CoreTelemetry];
    [NSThread sleepForTimeInterval: 0.5];
    [self sendPacket_ExtendedTelemetry];
    [NSThread sleepForTimeInterval: 0.5];
}

- (void) executeVirtualStickCommand_ModeA: (DroneInterface::VirtualStickCommand_ModeA *) command {
    DJIFlightController* fc = [DJIUtils fetchFlightController];
    [fc setRollPitchCoordinateSystem:DJIVirtualStickFlightCoordinateSystemGround];
    
    DJIVirtualStickFlightControlData ctrlData;
    ctrlData.yaw = command->Yaw;
    ctrlData.roll = command->V_North;
    ctrlData.pitch = command->V_East;
    ctrlData.verticalThrottle = command->HAG;
    
    if (fc.isVirtualStickControlModeAvailable) {
        [fc sendVirtualStickFlightControlData:ctrlData withCompletion:^(NSError * _Nullable error) {
            
        }];
    } else {
        //https://developer.dji.com/api-reference/ios-api/Components/FlightController/DJIFlightController.html#djiflightcontroller_virtualstickcontrolmodecategory_isvirtualstickcontrolmodeavailable_inline
        NSLog(@"Virtual stick control mode is not available in the current flight conditions. See documentation for details.");
    }
}

- (void) executeVirtualStickCommand_ModeB: (DroneInterface::VirtualStickCommand_ModeB *) command {
    DJIFlightController* fc = [DJIUtils fetchFlightController];
    [fc setRollPitchCoordinateSystem:DJIVirtualStickFlightCoordinateSystemBody];

    DJIVirtualStickFlightControlData ctrlData;
    ctrlData.yaw = command->Yaw;
    ctrlData.roll = command->V_Forward;
    ctrlData.pitch = command->V_Right;
    ctrlData.verticalThrottle = command->HAG;
    
    if (fc.isVirtualStickControlModeAvailable) {
        [fc sendVirtualStickFlightControlData:ctrlData withCompletion:^(NSError * _Nullable error) {
            
        }];
    } else {
        //https://developer.dji.com/api-reference/ios-api/Components/FlightController/DJIFlightController.html#djiflightcontroller_virtualstickcontrolmodecategory_isvirtualstickcontrolmodeavailable_inline
        NSLog(@"Virtual stick control mode is not available in the current flight conditions. See documentation for details.");
    }
}

#pragma mark - DJIMutableWaypointMission
- (void) createDJIWaypointMission: (DroneInterface::WaypointMission *) mission {
    if (self->_waypointMission) {
        [self->_waypointMission removeAllWaypoints];
    } else {
        self->_waypointMission = [[DJIMutableWaypointMission alloc] init];
    }
    
    for (int i = 0; i < mission->Waypoints.size(); i++) {
        CLLocation* location = [[CLLocation alloc] initWithLatitude:mission->Waypoints[i].Latitude longitude:mission->Waypoints[i].Longitude];
        
        if (CLLocationCoordinate2DIsValid(location.coordinate)) {
            DJIWaypoint* waypoint = [[DJIWaypoint alloc] initWithCoordinate:location.coordinate];
            waypoint.altitude = mission->Waypoints[i].Altitude;
            waypoint.cornerRadiusInMeters = mission->Waypoints[i].CornerRadius;
            waypoint.speed = mission->Waypoints[i].Speed;
            if (!isnan(mission->Waypoints[i].LoiterTime)) {
                [waypoint addAction:[[DJIWaypointAction alloc] initWithActionType:DJIWaypointActionTypeStay param:mission->Waypoints[i].LoiterTime]];
            }
            if (!isnan(mission->Waypoints[i].GimbalPitch)) {
                [waypoint addAction:[[DJIWaypointAction alloc] initWithActionType:DJIWaypointActionTypeRotateGimbalPitch param:mission->Waypoints[i].GimbalPitch]];
            }
            [self->_waypointMission addWaypoint:waypoint];
        } else {
            [self sendPacket_MessageString:@"Invalid waypoint coordinate." ofType:3];
        }
    }
    
    if (mission->LandAtLastWaypoint == 1) {
        self->_waypointMission.finishedAction = DJIWaypointMissionFinishedAutoLand;
    } else {
        self->_waypointMission.finishedAction = DJIWaypointMissionFinishedNoAction;
    }
    
    if (mission->CurvedTrajectory == 1) {
        self->_waypointMission.flightPathMode = DJIWaypointMissionFlightPathCurved;
    } else {
        self->_waypointMission.flightPathMode = DJIWaypointMissionFlightPathNormal;
    }
}

- (DJIWaypointMissionOperator *)missionOperator {
    return [DJISDKManager missionControl].waypointMissionOperator;
}

- (void) startDJIWaypointMission {
    [[self missionOperator] startMissionWithCompletion:^(NSError * _Nullable error) {
        
    }];
}

- (void) stopDJIWaypointMission {
    [[self missionOperator] stopMissionWithCompletion:^(NSError * _Nullable error) {
        
    }];
}

- (void) executeDJIWaypointMission: (DroneInterface::WaypointMission *) mission {
    [self createDJIWaypointMission:mission];
    
    [[self missionOperator] loadMission: self->_waypointMission];
    
    // On mission upload
    [[self missionOperator] uploadMissionWithCompletion:^(NSError * _Nullable error) {
        [self startDJIWaypointMission];
    }];
    
    // On mission finished
    [[self missionOperator] addListenerToFinished:self withQueue:dispatch_get_main_queue() andBlock:^(NSError * _Nullable error) {

    }];
}

@end
