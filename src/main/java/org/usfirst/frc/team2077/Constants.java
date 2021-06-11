/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

public final class Constants {

    public final double UPPER_TARGET_HEIGHT = 98.25; // inches from floor to center of target
    public final double LOWER_TARGET_HEIGHT = 23.; // inches from floor to center of target

    public final double AZIMUTH_CROSSHAIR_SENSITIVITY = 0.5; // joystick multiplier
    public final double RANGE_CROSSHAIR_SENSITIVITY = 2.0 * .875 ; // joystick multiplier

    public final double FISHEYE_CAMERA_TILT = 0; // degrees upward tilt
    public final double FISHEYE_CAMERA_HEIGHT = 17.; // inches from floor to camera lens (check this)
    public final double FISHEYE_CAMERA_POSITION_NORTH = 15.; // inches forward from chassis center to camara lens
    // following are for horizintal cylindrical projection as of 2/25
    public final int FISHEYE_CAMERA_PIXEL_HEIGHT = 1000;
    public final int FISHEYE_CAMERA_PIXEL_WIDTH = 1000;
    public final int FISHEYE_CAMERA_FOCAL_LENGTH = 334; // azimuth only (elevation is linear y pixels from center times .13)

    public final double INFRARED_MIN_LOADED = 1.4;
    public final double INFRARED_MAX_LOADED = 4;
}
