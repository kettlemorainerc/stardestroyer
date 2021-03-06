package org.usfirst.frc.team2077.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usfirst.frc.team2077.Robot.*;

public class DummyLauncher extends SubsystemBase implements LauncherIF {

    private final double launcherHeight_;
    private final Servo pointer_;

    public DummyLauncher(double launcherHeight) {
        launcherHeight_ = launcherHeight;
        pointer_ = new Servo(0); // PWM 0
    }

    @Override
    public boolean setRangeUpper(double range) {
        return setRange(range, robot_.constants_.UPPER_TARGET_HEIGHT);
    }

    @Override
    public boolean setRangeLower(double range) {
        return setRange(range, robot_.constants_.LOWER_TARGET_HEIGHT);
    }

    boolean debug_ = false;

    /**
     * Set launcher to a range and target height.
     * This example assumes a perfect trajectory and direct launch angle setting.
     * The real-world ball will use a lookup table to map range to mechanism settings.
     * @param range
     * @param height
     * @return True if target is within range.
     */
    private boolean setRange(double range, double height) {
      

        if (range <= 0) {
            //return false;
        }

        double targetHeight = height - launcherHeight_;

        double elevation = Math.toDegrees(Math.atan2(targetHeight, range)); // TODO: Compensate for launcher N
        pointer_.setAngle(elevation + 8);

        //if (debug_) System.out.println("ELEVATION: " + elevation);    // servo zero error     

        return true;
    }

    @Override
    public void setRunning(boolean running) {
    }

    @Override
    public boolean isLoaded() {
        return true;
    }

    @Override
    public void launch() {
    }

    @Override
    public String toString() {
        return "ELEVATION:" + Math.round(pointer_.getAngle());
    }

    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    public boolean isRunning() {
        return true;
    }

    @Override
    public void load() {
        // TODO Auto-generated method stub

    }
}
