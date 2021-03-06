package org.usfirst.frc.team2077.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Ball launcher. Wraps hardware-specific calculations for shooting at various ranges,
 * and motors, controllers, sensors, etc. necessary for actually making the shot.
 * Horizontal (azimuth) aiming is presumed to be handled by drivetrain rotation.
 * Ball loading is presumed to be handled by its own subsystem.
 */
public interface LauncherIF extends Subsystem {

    /**
     * Adjust elevation and shooter RPM for maximum hit probability on upper target at a given range.
     * @param range Distance from chassis center to target center in inches.
     * @return True if target is within range.
     */
    boolean setRangeUpper(double range);

    /**
     * Adjust elevation for maximum hit probability on lower target at a given range.
     * @param range Distance from chassis center to target center in inches.
     * @return True if target is within range.
     */
    boolean setRangeLower(double range);

    /**
     * Spin up the launcher mechanism.
     * @param running
     */
    void setRunning(boolean running);

    /**
     * @return True of launcher is spinning.
     */
    boolean isRunning();
    
    /**
     * Move a ball into launching position.
     */
    void load();

    /**
    * @return True if there is a ball ready to be launched.
     */
    boolean isLoaded();

    /**
     * @return True if running and up to correct speed for range.
     */
    boolean isReady();
    
    /**
     * Launch one ball.
     */
    void launch();
}
