/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

public class DummyDriveModule implements DriveModuleIF {

    // speed limit
    private final double maximumSpeed_;

    // speed set point
    private double speed_ = 0;

    // accumulated distance since last velocity change or distance reset
    private double distance_ = 0;

    // time of last distance reset
    private long distanceTime_ = System.currentTimeMillis();

    public DummyDriveModule(double maximumSpeed) {
        maximumSpeed_ = maximumSpeed;
    }

    public DummyDriveModule() {
        this(100);
    }

    @Override
    public double getMaximumSpeed() {
        return maximumSpeed_;
    }

    @Override
    public void setVelocity(double speed) {
        long now = System.currentTimeMillis();
        distance_ += speed_ * ((now-distanceTime_)/1000.);
        distanceTime_ = now;
        speed_ = speed;
    }

    @Override
    public WheelPosition getWheelPosition() {
        return null;
    }

    @Override
    public double getVelocity() {
        return speed_;
    }

    @Override
    public double getDistance() {
        long now = System.currentTimeMillis();
        return distance_ + speed_ * ((now-distanceTime_)/1000.);
    }

    @Override
    public void resetDistance() {
        distance_ = 0;
        distanceTime_ = System.currentTimeMillis();
    }

    @Override
    public String toString() {
        return "" + Math.round(100.*speed_)/100. + "(" + Math.round(100.*getVelocity())/100. + ")";
    }
}