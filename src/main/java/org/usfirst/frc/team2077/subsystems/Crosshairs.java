/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usfirst.frc.team2077.Robot.*;

public class Crosshairs extends SubsystemBase {


    private static double MINIMUM_RANGE = 18; // inches // TODO: Configure in Constants;
    private static double MAXIMUM_RANGE = 54*12; // inches // TODO: Configure in Constants;

    private double azimuth_ = 0;
    private double range_ = 50*12; // TODO: Don't rely on these initial values.


    public final void set(double azimuth, double range) {
        azimuth_ = normalize(azimuth);
        range_ = constrain(range);
    }

    public double[] get() {
        return new double[] {azimuth_, range_};
    }

    public double getAzimuth() {
        return azimuth_;
    }

    public double getRange() {
        return range_;
    }

    private static double constrain(double range) {
        return Math.max(robot_.constants_.MINIMUM_TARGET_RANGE, Math.min(robot_.constants_.MAXIMUM_TARGET_RANGE, range));
    }

    private static double normalize(double azimuth) {
        return (((azimuth % 360) + 360 + 180) % 360) - 180;
    }

    @Override
    public String toString() {
        return "Azimuth:" + Math.round(azimuth_*10.)/10. + " degrees, Range:" + Math.round(range_*10.)/10. + " inches";
    }

    /**
     * Test code.
     * May be run locally in VSCode.
     */
    public static void main(String[] argv) {

        System.out.println("" + normalize(361) + " (1);");
        System.out.println("" + normalize(-361) + " (-1);");
        System.out.println("" + normalize(359) + " (-1);");
        System.out.println("" + normalize(-359) + " (1);");
        System.out.println("" + normalize(181) + " (-179);");
        System.out.println("" + normalize(-181) + " (179);");
        System.out.println("" + normalize(179) + " (179);");
        System.out.println("" + normalize(-179) + " (-179);");
    }








    private double horizontalFOV_ = 2. * Math.toDegrees(Math.atan2(robot_.constants_.FISHEYE_CAMERA_PIXEL_WIDTH/2., robot_.constants_.FISHEYE_CAMERA_FOCAL_LENGTH));

    private double pixelX_ = 0;
    private double pixelY_ = 0;


    public double getHorizontalFOV() {
        return horizontalFOV_;
    }

    public double[] getCamera() {
        targetToCamera();
        return new double[] {pixelX_, pixelY_, robot_.constants_.FISHEYE_CAMERA_PIXEL_WIDTH, robot_.constants_.FISHEYE_CAMERA_PIXEL_HEIGHT};
    }

    private void targetToCamera() {

        double azimuth = azimuth_;
        double range = range_;
        double north = range_ * Math.cos(Math.toRadians(azimuth_));
        double east = range_ * Math.sin(Math.toRadians(azimuth_));

        north -= robot_.constants_.FISHEYE_CAMERA_POSITION_NORTH;
        azimuth = Math.toDegrees(Math.atan2(east, north)); // from camera to target
        range = Math.sqrt(north*north + east*east); // from camera to target
        double elevation = Math.toDegrees(Math.atan2(robot_.constants_.UPPER_TARGET_HEIGHT - robot_.constants_.FISHEYE_CAMERA_HEIGHT, range)); // from ground to target
        elevation -= robot_.constants_.FISHEYE_CAMERA_TILT; // from camera axis to target
        pixelX_ = robot_.constants_.FISHEYE_CAMERA_FOCAL_LENGTH * Math.tan(Math.toRadians(Math.max(-horizontalFOV_/2., Math.min(horizontalFOV_/2., azimuth))));
        //pixelY_ = robot_.constants_.FISHEYE_CAMERA_FOCAL_LENGTH * Math.tan(Math.toRadians(elevation)); // perspective projection
        pixelY_ = elevation / 0.13; // horizontal cylindrical projection, 2/25 version
    }
}
