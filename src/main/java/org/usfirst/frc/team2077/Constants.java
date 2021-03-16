/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

public final class Constants {

    public final double UPPER_TARGET_HEIGHT = 98.25; // inches from floor to center of target
    public final double LOWER_TARGET_HEIGHT = 23.; // inches from floor to center of target
    public final double MINIMUM_TARGET_RANGE = 18.; // minimum possible distance from target
    public final double MAXIMUM_TARGET_RANGE = 60.*12.-18.; // maximum possible distance from target

    public final double AZIMUTH_CROSSHAIR_SENSITIVITY = 0.5; // joystick multiplier
    public final double RANGE_CROSSHAIR_SENSITIVITY = 2.0; // joystick multiplier

    public final double FISHEYE_CAMERA_TILT = 0; // degrees upward tilt
    public final double FISHEYE_CAMERA_HEIGHT = 17.; // inches from floor to camera lens (check this)
    public final double FISHEYE_CAMERA_POSITION_NORTH = 15.; // inches forward from chassis center to camara lens
    // following are for horizintal cylindrical projection as of 2/25
    public final int FISHEYE_CAMERA_PIXEL_HEIGHT = 1000;
    public final int FISHEYE_CAMERA_PIXEL_WIDTH = 1000;
    public final int FISHEYE_CAMERA_FOCAL_LENGTH = 334; // azimuth only (elevation is linear y pixels from center times .13)

    
    public final double STARDESTROYER_WHEELBASE = 20.375; // inches
    public final double STARDESTROYER_TRACK_WIDTH = 22.625; // inches
    public final double STARDESTROYER_WHEEL_RADIUS = 4.0; // inches
    public final double STARDESTROYER_SHOOTER_RADIUS = 2.0;
    // TODO: move motor RPM limit back up and use acceleration and/or stick scaling to optimize driveability
    public final double STARDESTROYER_MOTOR_RPM_LIMIT = 5500; // (82 inches/second)
    // TODO: use acceleration constants where appropriate throughout other code
    // TODO: different values for operator drive vs programmed motion vs short nudges, etc?
    public final double STARDESTROYER_ACCELERATION_G_LIMIT = .5;
    public final double STARDESTROYER_DECELERATION_G_LIMIT = .25;

    //for autonomous
    // public final double STARDESTROYER_ACCELERATION_G_LIMIT = .1;
    // public final double STARDESTROYER_DECELERATION_G_LIMIT = .3;

    public final double INFRARED_MIN_LOADED = 1.4;
    public final double INFRARED_MAX_LOADED = 4;

    public final double PIZZABOT_WHEELBASE = 17.5; // inches 
    public final double PIZZABOT_TRACK_WIDTH = 18.0; // inches 
    public final double PIZZABOT_WHEEL_RADIUS = 3.0; // inches
    public final double PIZZABOT_MOTOR_RPM_LIMIT = 4000; // maximum speed may be limited by RPM or inches/second

    public final double PIZZABOT_CAMERA_HEIGHT = 17.5; // inches
    public final double PIZZABOT_CAMERA_TILT = 25.5; // degrees above vertical
    public final double PIZZABOT_CAMERA_POSITION_NORTH = 2.; // inches forward from chassis center to camara lens

    public final double PIZZABOT_LAUNCHER_POSITION_NORTH = 2.; // inches forward from chassis center to camara lens
    public final double PIZZABOT_LAUNCHER_HEIGHT = 12.0; // inches (approximate)
}
