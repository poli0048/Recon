//
//  DJIUtils.m
//  DroneClient
//
//  Created by Ben Choi on 3/31/21.
//

#import "DJIUtils.h"

@implementation DJIUtils

+ (NSString *)formattingSeconds:(NSUInteger)seconds
{
    NSDate *date = [NSDate dateWithTimeIntervalSince1970:seconds];
    NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
    [formatter setDateFormat:@"mm:ss"];
    [formatter setTimeZone:[NSTimeZone timeZoneForSecondsFromGMT:0]];
    
    NSString *formattedTimeString = [formatter stringFromDate:date];
    return formattedTimeString;
}

+ (DJICamera*) fetchCamera {
    
    if (![DJISDKManager product]) {
        return nil;
    }
    
    if ([[DJISDKManager product] isKindOfClass:[DJIAircraft class]]) {
        return ((DJIAircraft*)[DJISDKManager product]).camera;
    }else if ([[DJISDKManager product] isKindOfClass:[DJIHandheld class]]){
        return ((DJIHandheld *)[DJISDKManager product]).camera;
    }
    
    return nil;
}

+ (DJIFlightController*) fetchFlightController {
    if (![DJISDKManager product]) {
        return nil;
    }
    
    if ([[DJISDKManager product] isKindOfClass:[DJIAircraft class]]) {
        return ((DJIAircraft*)[DJISDKManager product]).flightController;
    }
    
    return nil;
}

+ (DJIBattery*) fetchBattery {
    if (![DJISDKManager product]) {
        return nil;
    }
    
    if ([[DJISDKManager product] isKindOfClass:[DJIAircraft class]]) {
        return ((DJIAircraft*)[DJISDKManager product]).battery;
    }
    
    return nil;
}

+ (bool)gpsStatusIsGood:(DJIGPSSignalLevel)signalStatus {
    switch (signalStatus) {
        case DJIGPSSignalLevel5:
            return YES;
        case DJIGPSSignalLevel4:
            return YES;
        case DJIGPSSignalLevel3:
        case DJIGPSSignalLevel2:
        case DJIGPSSignalLevel1:
        case DJIGPSSignalLevel0:
        case DJIGPSSignalLevelNone:
        default:
            return NO;
    }
}

+ (int8_t)getGNSSSignal:(DJIGPSSignalLevel) signalStatus {
    switch (signalStatus) {
        case DJIGPSSignalLevel5:
            return 5;
        case DJIGPSSignalLevel4:
            return 4;
        case DJIGPSSignalLevel3:
            return 3;
        case DJIGPSSignalLevel2:
            return 2;
        case DJIGPSSignalLevel1:
            return 1;
        case DJIGPSSignalLevel0:
            return 0;
        case DJIGPSSignalLevelNone:
        default:
            return -1;
    }
}

+ (int8_t)getWindLevel:(DJIFlightWindWarning) windWarning {
    switch (windWarning) {
        case DJIFlightWindWarningLevel2:
            return 2;
        case DJIFlightWindWarningLevel1:
            return 1;
        case DJIFlightWindWarningLevel0:
            return 0;
        case DJIFlightWindWarningUnknown:
        default:
            return -1;
    }
}

+ (UInt8)getFlightMode:(DJIFlightMode) flightMode {
    switch (flightMode) {
        case DJIFlightModeManual:
            return 0;
        case DJIFlightModeAtti:
            return 1;
        case DJIFlightModeAttiCourseLock:
            return 2;
        case DJIFlightModeGPSAtti:
            return 3;
        case DJIFlightModeGPSCourseLock:
            return 4;
        case DJIFlightModeGPSHomeLock:
            return 5;
        case DJIFlightModeGPSHotPoint:
            return 6;
        case DJIFlightModeAssistedTakeoff:
            return 7;
        case DJIFlightModeAutoTakeoff:
            return 8;
        case DJIFlightModeAutoLanding:
            return 9;
        case DJIFlightModeGPSWaypoint:
            return 10;
        case DJIFlightModeGoHome:
            return 11;
        case DJIFlightModeJoystick:
            return 12;
        case DJIFlightModeGPSAttiWristband:
            return 13;
        case DJIFlightModeDraw:
            return 14;
        case DJIFlightModeGPSFollowMe:
            return 15;
        case DJIFlightModeActiveTrack:
            return 16;
        case DJIFlightModeTapFly:
            return 17;
        case DJIFlightModeGPSSport:
            return 18;
        case DJIFlightModeGPSNovice:
            return 19;
        case DJIFlightModeUnknown:
            return 20;
        case DJIFlightModeConfirmLanding:
            return 21;
        case DJIFlightModeTerrainFollow:
            return 22;
        case DJIFlightModeTripod:
            return 23;
        case DJIFlightModeActiveTrackSpotlight:
            return 24;
        case DJIFlightModeMotorsJustStarted:
            return 25;
        default:
            return 255;
    }
}

@end
