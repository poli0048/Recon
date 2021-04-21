//
//  DJIUtils.h
//  DroneClient
//
//  Created by Ben Choi on 3/31/21.
//

#import <Foundation/Foundation.h>
#import <DJISDK/DJISDK.h>

#ifndef DJIUtils_h
#define DJIUtils_h


@interface DJIUtils : NSObject
+(NSString *)formattingSeconds:(NSUInteger) seconds;
+(DJIFlightController*) fetchFlightController;
+(DJIBattery*) fetchBattery;
+(DJICamera*) fetchCamera;
+(bool)gpsStatusIsGood:(DJIGPSSignalLevel) signalStatus;
+(int8_t)getGNSSSignal:(DJIGPSSignalLevel) signalStatus;
+(int8_t)getWindLevel:(DJIFlightWindWarning) windWarning;
+(UInt8)getFlightMode:(DJIFlightMode) flightMode;
@end


#endif /* DJIUtils_h */
