/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.Timer;

/**
 * Timing clock. Wraps {@link Timer#getFPGATimestamp} if available,
 * or {@link System#nanoTime} otherwise, to support testing code
 * without RoboRio hardware.
 */
public final class Clock {
    private static final double NS_PER_SECOND = 1000000000.;
    private static final long nanoTimeBase_ = System.nanoTime();

    private Clock() {} // Use static API only.

    /**
     * @return Seconds since an indeterminate but consistent point in time.
     */
    public static double getSeconds() {
        return Robot.robot_ != null ? Timer.getFPGATimestamp() : ((System.nanoTime()-nanoTimeBase_) / NS_PER_SECOND);
    }
}
