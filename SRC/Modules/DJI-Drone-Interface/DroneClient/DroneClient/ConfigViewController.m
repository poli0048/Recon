//
//  ConfigViewController.m
//  DroneClient
//
//  Created by Ben Choi on 2/17/21.
//

#import "ConfigViewController.h"
#import "ConnectionController.h"
#import <DJISDK/DJISDK.h>
#import "Constants.h"

@interface ConfigViewController ()<DJISDKManagerDelegate>

@end

@implementation ConfigViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    UITapGestureRecognizer *tapGestureRecognizer=[[UITapGestureRecognizer alloc] initWithTarget:self.view action:@selector(endEditing:)];
    [tapGestureRecognizer setCancelsTouchesInView:NO];
    [self.view addGestureRecognizer:tapGestureRecognizer];
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender{
    if([segue.identifier isEqualToString:@"connect"]){
        ConnectionController *controller = (ConnectionController *)segue.destinationViewController;
        controller->ipAddress = _ipAddressTextField.text;
        controller->port = _portTextField.text;
    }
}

@end
