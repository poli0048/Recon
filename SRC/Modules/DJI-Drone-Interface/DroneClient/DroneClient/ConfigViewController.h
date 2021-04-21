//
//  ConfigViewController.h
//  DroneClient
//
//  Created by Ben Choi on 2/17/21.
//

#import <UIKit/UIKit.h>

#ifndef ConfigViewController_h
#define ConfigViewController_h


#endif /* ConfigViewController_h */

@interface ConfigViewController : UIViewController<NSStreamDelegate>
{

}

@property (weak, nonatomic) IBOutlet UITextField *ipAddressTextField;
@property (weak, nonatomic) IBOutlet UITextField *portTextField;


@end
