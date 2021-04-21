#ifndef DJIDecodeImageCalibrateControlLogic_h
#define DJIDecodeImageCalibrateControlLogic_h

#import <DJIWidget/DJIImageCalibrateFilterDataSource.h>


@interface DJIDecodeImageCalibrateControlLogic : NSObject <DJIImageCalibrateDelegate>

@property (nonatomic, assign) NSUInteger cameraIndex;
@property (nonatomic, copy) NSString* cameraName;

@end


#endif /* DJIDecodeImageCalibrateControlLogic_h */
