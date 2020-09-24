#include <algorithm>
#include <future>
#include <iostream>
#include <vector>
#include <cmath>
#include <termios.h>

#include "rofi_hal.hpp"


using namespace rofi::hal;

struct rofi_arm
{
    // easier access to rofis and joints
    std::vector< RoFI > rofis;
    std::vector< std::vector< Joint > > joints;

    rofi_arm( int length ){
        for( int i = 1; i <= length; i++ ){
            rofis.push_back( RoFI::getRemoteRoFI( i ) );

            joints.push_back( { RoFI::getRemoteRoFI( i ).getJoint( 0 ),
                                RoFI::getRemoteRoFI( i ).getJoint( 1 ),
                                RoFI::getRemoteRoFI( i ).getJoint( 2 ) } );
        }
    }
};

void parseInput(rofi_arm arm){
    char c;

    double maxPos = arm.joints[ 0 ][ 0 ].maxPosition();
    // double minPos = arm.joints[ 0 ][ 0 ].minPosition();
    double maxSpeed = arm.joints[ 0 ][ 0 ].maxSpeed();

    double newPos = 0;

	do {
        c = getchar();

        std::promise< void > promise;
        auto future = promise.get_future();

        switch( c ){
            case 'w':
                newPos = std::clamp( arm.joints[ 0 ][ 1 ].getPosition() + 0.1, 0.0, maxPos );
                arm.joints[ 0 ][ 1 ].setPosition( newPos, maxSpeed, [ & ]( Joint ){ promise.set_value(); } );
                break;
            case 's':
                newPos = std::clamp( arm.joints[ 0 ][ 1 ].getPosition() - 0.1, 0.0, maxPos );
                arm.joints[ 0 ][ 1 ].setPosition( newPos, maxSpeed, [ & ]( Joint ){ promise.set_value(); } );
                break;
            case 'a':
                newPos = std::clamp( arm.joints[ 0 ][ 2 ].getPosition() + 0.1, - M_PI / 2, M_PI / 2 );
                arm.joints[ 0 ][ 2 ].setPosition( newPos, maxSpeed, [ & ]( Joint ){ promise.set_value(); } );
                break;
            case 'd':
                newPos = std::clamp( arm.joints[ 0 ][ 2 ].getPosition() - 0.1, - M_PI / 2, M_PI / 2 );
                arm.joints[ 0 ][ 2 ].setPosition( newPos, maxSpeed, [ & ]( Joint ){ promise.set_value(); } );
                break;
            default:
                continue;
        }
        future.get();

	} while( c != 'q' );
	
}

int main()
{
    std::cout << "Connecting\n";

    rofi_arm arm( 3 );

    double maxPos = arm.joints[ 0 ][ 0 ].maxPosition();
    double minPos = arm.joints[ 0 ][ 0 ].minPosition();
    double maxSpeed = arm.joints[ 0 ][ 0 ].maxSpeed();

    // unbuffer terminal input to control the arm
    struct termios old_tio, new_tio;
	tcgetattr( STDIN_FILENO, &old_tio );
	new_tio = old_tio;
	new_tio.c_lflag &= ( ~ICANON & ~ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &new_tio );

    std::promise< void > promise1, promise2, promise3, promise4;
    auto future1 = promise1.get_future();
    auto future2 = promise2.get_future();
    auto future3 = promise3.get_future();
    auto future4 = promise4.get_future();

    // world creator doesn't support the static connector, wait for it to manually connect
    while( !arm.rofis[ 0 ].getConnector( 2 ).getState().connected ){
        std::this_thread::sleep_for( std::chrono::seconds( 1 ));
    }

    arm.joints[ 0 ][ 0 ].setPosition( maxPos / 2, maxSpeed, [ & ]( Joint ){
        arm.joints[ 0 ][ 0 ].setPosition( maxPos, maxSpeed, [ & ]( Joint ){ promise1.set_value(); } );
    } );
    arm.joints[ 0 ][ 1 ].setPosition( minPos / 2, maxSpeed, [ & ]( Joint ){
        arm.joints[ 0 ][ 1 ].setPosition( 0, maxSpeed, [ & ]( Joint ){ promise2.set_value(); } );
    } );
    arm.joints[ 1 ][ 0 ].setPosition( minPos / 2, maxSpeed, [ & ]( Joint ){
        arm.joints[ 1 ][ 0 ].setPosition( minPos, maxSpeed, [ & ]( Joint ){ promise3.set_value(); } );
    } );
    arm.joints[ 1 ][ 1 ].setPosition( minPos / 2, maxSpeed, [ & ]( Joint ){
        arm.joints[ 1 ][ 1 ].setPosition( minPos, maxSpeed, [ & ]( Joint ){ promise4.set_value(); } );
    } );
    arm.joints[ 2 ][ 0 ].setPosition( -0.1, maxSpeed, nullptr );
  
    future1.get();
    future2.get();
    future3.get();
    future4.get();

    parseInput( arm );

	tcsetattr( STDIN_FILENO, TCSANOW, &old_tio );
    
    return 0;
}
