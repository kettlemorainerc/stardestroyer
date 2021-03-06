/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import static org.usfirst.frc.team2077.Robot.*;

public class MecanumChassis extends AbstractChassis {

    private final MecanumMath mecanumMath_;

    /**
     * @param driveModule [4]
     * @param wheelbase
     * @param trackWidth
     * @param wheelRadius
     */
    public MecanumChassis(DriveModuleIF[] driveModule, double wheelbase, double trackWidth, double wheelRadius) {
    
        super(driveModule, wheelbase, trackWidth, wheelRadius);

        mecanumMath_ = new MecanumMath(wheelbase_, trackWidth_, wheelRadius_, wheelRadius_, 1, 180/Math.PI);

        // north/south speed conversion from 0-1 range to DriveModule maximum (inches/second)
        maximumSpeed_ = Math.min(
            Math.min(driveModule_[0].getMaximumSpeed(), driveModule_[1].getMaximumSpeed()),
            Math.min(driveModule_[2].getMaximumSpeed(), driveModule_[3].getMaximumSpeed()));     
        // rotation speed conversion from 0-1 range to DriveModule maximum (degrees/second)
        maximumRotation_ = mecanumMath_.forward(new double[] {-maximumSpeed_, -maximumSpeed_, maximumSpeed_, maximumSpeed_})[2];
        
        // lowest chassis speeds supportable by the drive modules
        minimumSpeed_ = maximumSpeed_ * .1; // TODO: Test and configure.
        minimumRotation_ = maximumRotation_ * .1;

        System.out.println(getClass().getName() + "MAXIMUM SPEED:" + Math.round(maximumSpeed_*10.)/10. + " IN/SEC MAXIMUM ROTATION:" + Math.round(maximumRotation_*10.)/10. + " DEG/SEC");
        System.out.println(getClass().getName() + "MINIMUM SPEED:" + Math.round(minimumSpeed_*10.)/10. + " IN/SEC MINIMUM ROTATION:" + Math.round(minimumRotation_*10.)/10. + " DEG/SEC");
        double[][] a = getAccelerationLimits();
        System.out.println(getClass().getName() + "ACCELERATION:"
            + Math.round(a[0][0]*10.)/10. + "/" + Math.round(a[0][1]*10.)/10. + "/"
            + Math.round(a[1][0]*10.)/10. + "/" + Math.round(a[1][1]*10.)/10. + "/"
            + Math.round(a[2][0]*10.)/10. + "/" + Math.round(a[2][1]*10.)/10.);
    }

    @Override
    public void setVelocity(double north, double east, double clockwise, double[][] accelerationLimits) {
        northSet_ = north;
        eastSet_ = east;
        clockwiseSet_ = clockwise;
        northAccelerationLimit_ = accelerationLimits[0];
        eastAccelerationLimit_ = accelerationLimits[1];
        rotationAccelerationLimit_ = accelerationLimits[2];
    }

    @Override
    public void setVelocity(double north, double east, double[][] accelerationLimits) {
        northSet_ = north;
        eastSet_ = east;
        northAccelerationLimit_ = accelerationLimits[0];
        eastAccelerationLimit_ = accelerationLimits[1];
     }

    @Override
    public void setRotation(double clockwise, double[][] accelerationLimits) {
        clockwiseSet_ = clockwise;
        rotationAccelerationLimit_ = accelerationLimits[2];
    }

    @Override
    public double[] getVelocitySet() {
        return new double[] {northSet_, eastSet_, clockwiseSet_};
    }

    @Override
    public double[] getVelocityCalculated() {
        return new double[] {north_, east_, clockwise_};
    }

    @Override
    public double[] getVelocityMeasured() {
        return new double[] {velocityMeasured_[0], velocityMeasured_[1], velocityMeasured_[2]};
    }

    @Override
    protected void updatePosition() {

        // chassis velocity from internal set point
        velocitySet_ = getVelocityCalculated();
        // chassis velocity from motor/wheel measurements
        double[] w = {driveModule_[0].getVelocity(), driveModule_[1].getVelocity(), driveModule_[2].getVelocity(), driveModule_[3].getVelocity()};
        velocityMeasured_ = mecanumMath_.forward(w);

        // TODO: E/W velocities are consistently lower than those calculated from wheel speeds.
        // TODO: Measure actual vs measured E/W distances and insert an adjustment factor here.
        // TODO: Put the adjustment factor in constants.
        velocitySet_[1] *= .65; // just a guess
        velocityMeasured_[1] *= .65; // just a guess
    
        // update position with motion since last update
        double dT = timeSinceLastUpdate_;
        positionSet_.moveRelative(velocitySet_[0] * dT, velocitySet_[1] * dT, velocitySet_[2] * dT);
        positionMeasured_.moveRelative(velocityMeasured_[0] * dT, velocityMeasured_[1] * dT, velocityMeasured_[2] * dT);
        if (robot_.angleSensor_ != null) { // TODO: Confirm AngleSensor is actually reading. Handle bench testing.
            double[] pS = positionSet_.get();
            double[] pM = positionMeasured_.get();
            pS[2] = pM[2] = robot_.angleSensor_.getAngle(); // TODO: conditional on gyro availability
            positionSet_.set(pS[0], pS[1], pS[2]);
            positionMeasured_.set(pM[0], pM[1], pM[2]);        
        }

        if (debug_) {
            double[] pS = positionSet_.get();
            double[] pM = positionMeasured_.get();
            System.out.print("POSITION:{"
                + Math.round(pS[0]*100.)/100. + "(" + Math.round(pM[0]*100.)/100. + "), "
                + Math.round(pS[1]*100.)/100. + "(" + Math.round(pM[1]*100.)/100. + "), "
                + Math.round(pS[2]*10.)/10. + "(" + Math.round(pM[2]*10.)/10. + ")}");
        }
    }

    @Override
    protected void updateDriveModules() {
        
        double[] v = getVelocityCalculated();
    //    if (debug_ ) System.out.print("CHASSIS: " + Math.round(v[0]*10.)/10. + " " + Math.round(v[1]*10.)/10. + " " + Math.round(v[2]*10.)/10.);
    
        // compute motor speeds
        double[] w = mecanumMath_.inverse(v);

        // scale all motors proportionally if any are out of range
        double max = 1;
        for (double ws : w) {
            max = Math.max(max, Math.abs(ws)/maximumSpeed_);
        }

    //    if (debug_ ) System.out.print(" WHEELS:");
        for (int i = 0; i < w.length; i++) {
            double ws = w[i] / max;
            driveModule_[i].setVelocity(ws);
    //        if (debug_ ) System.out.print(" " + Math.round(100.*ws)/100. + "(" + Math.round(100.*driveModule_[i].getVelocity())/100. + ")");
        }
        if (debug_ ) System.out.println(" " + this);
    }

    @Override
    public String toString() {
        return "V:" + Math.round(north_*10.)/10. + "/" + Math.round(east_*10.)/10. + "/" + Math.round(clockwise_*10.)/10.
            + " W:" + driveModule_[0] + "/" + driveModule_[1] + "/" + driveModule_[2] + "/" + driveModule_[3];
    }

    /**
     * Test code. May be run locally in VSCode. // TODO: Local execution broken due to Subsystem dependency.
     */
    public static void main(String[] argv) {

        // DriveChassisIF chassis;

        // DriveModuleIF[] module;

        // double wheelbase;
        // double trackWidth;
        // double wheelRadius;

        // chassis = new MecanumChassis(module = new DummyDriveModule[] {
        //     new DummyDriveModule(100),
        //     new DummyDriveModule(100),
        //     new DummyDriveModule(100),
        //     new DummyDriveModule(100)
        // },
        // wheelbase = 20,
        // trackWidth = 20,
        // wheelRadius = 3
        // );

        // System.out.println();

        // chassis.setVelocity01(.5, 0, 0);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity01(0, .5, 0);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity01(.5, .5, 0);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity01(0, 0, .5);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity01(.5, 0, .5);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity01(0, .5, .5);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity01(.5, .5, .5);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity(0, 0, -360);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setRotation(360);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity(70, 70, 0);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity(-50,-50, -90);
        // System.out.println(chassis);
        // System.out.println();


    }
}
