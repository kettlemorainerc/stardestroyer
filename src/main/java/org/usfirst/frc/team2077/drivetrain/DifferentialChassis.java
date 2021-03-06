/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import static org.usfirst.frc.team2077.Robot.*;

public class DifferentialChassis extends AbstractChassis {

    private final DifferentialMath differentialMath_;

    /**
     * @param driveModule [2]
     * @param trackWidth
     * @param wheelRadius
     */
    public DifferentialChassis(DriveModuleIF[] driveModule, double wheelbase, double trackWidth, double wheelRadius) {
    
        super(driveModule, wheelbase, trackWidth, wheelRadius);

        differentialMath_ = new DifferentialMath(trackWidth_, wheelRadius_);

        // north/south speed conversion from 0-1 range to DriveModule maximum (inches/second)
        maximumSpeed_ = Math.min(driveModule_[0].getMaximumSpeed(), driveModule_[1].getMaximumSpeed());
        // rotation speed conversion from 0-1 range to DriveModule maximum (degrees/second)
        maximumRotation_ = differentialMath_.forward(new double[]{-maximumSpeed_, maximumSpeed_})[1];

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
        clockwiseSet_ = clockwise;
        northAccelerationLimit_ = accelerationLimits[0];
        rotationAccelerationLimit_ = accelerationLimits[2];
    }

    @Override
    public void setVelocity(double north, double east, double[][] accelerationLimits) {
        northAccelerationLimit_ = accelerationLimits[0];
        northSet_ = north;
    }

    @Override
    public void setRotation(double clockwise, double[][] accelerationLimits) {
        clockwiseSet_ = clockwise;
        rotationAccelerationLimit_ = accelerationLimits[2];
    }

    @Override
    public double[] getVelocitySet() {
        return new double[] {northSet_, 0, clockwiseSet_};
    }

    @Override
    public double[] getVelocityCalculated() {
        return new double[] {north_, 0, clockwise_};
    }

    @Override
    public double[] getVelocityMeasured() {
        return getVelocityCalculated();
    }

    @Override
    protected void updatePosition() {

        // time since last update
        double dT = timeSinceLastUpdate_;

        // chassis velocity from internal set point
        double[] vS = getVelocityCalculated();
        // chassis velocity from motor/wheel measurements
        double[] w = {driveModule_[0].getVelocity(), driveModule_[1].getVelocity()};
        double[] v = differentialMath_.forward(w);
        double[] vM = {v[0], 0, v[1]};

        // update position with motion since last update
        positionSet_.moveRelative(vS[0]*dT, vS[1]*dT, vS[2]*dT);
        positionMeasured_.moveRelative(vM[0]*dT, vM[1]*dT, vM[2]*dT);
        if (robot_.angleSensor_ != null) { // TODO: Confirm AngleSensor is actually reading. Handle bench testing.
            double[] pS = positionSet_.get();
            double[] pM = positionMeasured_.get();
            pS[2] = pM[2] = robot_.angleSensor_.getAngle();
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
    
        double[] v = {north_, clockwise_};
        if (debug_ ) System.out.print(" DIFFERENTIAL INPUT: " + v[0] + " " + v[1]);

        // compute motor speeds
        double[] w = differentialMath_.inverse(v);

        // scale all motors proportionally if any are out of range
        double max = 1;
        for (double ws : w) {
            max = Math.max(max, Math.abs(ws)/maximumSpeed_);
        }

        if (debug_ ) System.out.print(" DRIVE MODULE INPUT:");
        for (int i = 0; i < w.length; i++) {
            double ws = w[i] / max;
            driveModule_[i].setVelocity(ws);
            if (debug_ ) System.out.print(" " + Math.round(100.*ws)/100. + "(" + Math.round(100.*driveModule_[i].getVelocity())/100. + ")");

        }
        if (debug_ ) System.out.println();
    }

    @Override
    public String toString() {
        return "V:" + north_ + "/" + clockwise_ + " W:" + driveModule_[0] + "/" + driveModule_[1];
    }

    /**
     * In the pattern of MecanumMath.
     */ 
    private static class DifferentialMath {

        private final double chassisRadius_;

        private final double wheelRadius_;

        /**
         * Calls to {@link #inverse(double[])} or {@link #forward(double[])} will calculate translation motions
         * using the same length units as width and wheel radius, with robot and wheel rotations in radians.
         * @param width Distance between wheel contact points in the E/W direction, in any distance unit.
         * @param wheelRadius Radius of each wheel, in the same unit.
         */
        public DifferentialMath(double width, double wheelRadius) {

            chassisRadius_ = width/2;
            wheelRadius_ = wheelRadius;
        }

        /***
         * @param V Chassis motion [north/south translation, rotation], in inches and degrees respectively.
         * @return Wheel speeds [E, W] in inches.
         */
        public final double[] inverse(double[] V) {

            double translation = V[0]; // in wheel radians
            double rotation = V[1] / (180/Math.PI) / (wheelRadius_/chassisRadius_) * wheelRadius_; // in wheel radians
            return new double[] {translation - rotation, translation + rotation};
        }

        /***
         * @param O Wheel speeds [E, W] in inches.
         * @return Chassis motion [north/south translation, rotation] in inches and degrees respectively.
         */
        public final double[] forward(double[] O) {

            double translation = (O[0] + O[1]) / 2; // in inches
            double rotation = (O[1] - O[0]) / 2; // in inches
 
            return new double[] {translation, rotation / wheelRadius_ * (wheelRadius_/chassisRadius_) * (180/Math.PI) };
        }
    }

    /**
     * Test code. May be run locally in VSCode. (?)
     */
    public static void main(String[] argv) {

        // DriveChassisIF chassis;

        // DriveModuleIF[] module;

        // double wheelbase;
        // double trackWidth;
        // double wheelRadius;

        // chassis = new DifferentialChassis(module = new DummyDriveModule[] {
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

        // chassis.setVelocity01(0, 0, .5);
        // System.out.println(chassis);
        // System.out.println();

        // chassis.setVelocity(0, 0, -360);
        // System.out.println(chassis);
        // System.out.println();
    }
}
