package org.usfirst.frc.team2077.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.*;

import static org.usfirst.frc.team2077.Robot.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import jdk.internal.loader.Loader;

public class TestGrabber extends SubsystemBase {
    private final VictorSPX input_;
    private final VictorSPX elevator_;
    private byte balls = 0;
    private boolean wasTLoaded = false;
    private boolean wasBLoaded = false; 
    private double liftSpeed = 0;
    private double inputSpeed = 0;
    private boolean loading = false;
    private boolean InBetween = false;
    private boolean emptyT = false;
    private boolean emptyB = false;

    public TestGrabber() {
        input_ = new VictorSPX(6);
        elevator_ = new VictorSPX(4);
    } 

    public void periodic() { // continuous adjustment of screw motor toward setpoint
        boolean Tloaded = isTLoaded();
        boolean Bloaded = isBLoaded();
        boolean firing = robot_.launcher_.isRunning();
        boolean newBottom = Bloaded && !wasBLoaded;
        boolean newTop = Tloaded && !wasTLoaded;
        // System.out.println("[TestGrabber]: " + balls);
        SmartDashboard.putNumber("voltage", robot_.infraredSensor_.getAverageVoltage());
        // SmartDashboard.putBoolean("switched", );
        //System.out.println(robot_.infraredSensor_.getAverageVoltage());
        if (newTop && InBetween) {
            InBetween = false;
        }
        if (newBottom) {
           InBetween = true;
        }
       
        if (balls < 3) { //if it can load more balls
            if (!Tloaded && wasTLoaded) {
                balls++;
            }
        } else if (balls == 3) { //4th ball coming 
            if (newTop) {
                balls++;
            }
        } else if (balls == 4) { //5th ball coming
            if (newBottom) { //if a new ball appeared on the bottom
                ++balls;
            }
        }

        if (loading) {
            if (balls < 3) { //if it can load more balls
                if (InBetween) {
                    //Run Motors at same speed until it's passed the T Sensor
                    input_.set(ControlMode.PercentOutput, -0.5); //TODO: Check speeds
                    elevator_.set(ControlMode.PercentOutput, -0.5);
                } else {
                //Run just the intake fast
                    input_.set(ControlMode.PercentOutput, -0.5);
                }
            
            } else if (balls == 3) { //4th ball coming 
                if (InBetween) {
                    //Run Motors at same speed until it's passed the T Sensor
                    input_.set(ControlMode.PercentOutput, -0.3); //TODO: Check speeds
                    elevator_.set(ControlMode.PercentOutput, -0.3);
                } else {
                    //Run just the intake fast
                    input_.set(ControlMode.PercentOutput, -0.5);
                }
            } else if (balls == 4) { //5th ball coming
                elevator_.set(ControlMode.PercentOutput, 0);
                input_.set(ControlMode.PercentOutput, -0.3); //TODO: Get speed for this
            } else { //all 5 balls
                input_.set(ControlMode.PercentOutput,0);
                elevator_.set(ControlMode.PercentOutput, 0);
                System.out.println("[TestGrabber]: STOP LIFT BALL MOTOR");
            }
        } else if (emptyT) { //Empties top before bottom
            if (isTLoaded()) {
                elevator_.set(ControlMode.PercentOutput, -0.8);
            } else {
                emptyT = false;
                elevator_.set(ControlMode.PercentOutput, 0);
            }
        } else if (emptyB) {
            if (isBLoaded() || isTLoaded()) {
                System.out.println(isBLoaded() + "" + isTLoaded());
                input_.set(ControlMode.PercentOutput, -0.8);
                elevator_.set(ControlMode.PercentOutput, -0.8);
            } else {
                input_.set(ControlMode.PercentOutput, 0);
                elevator_.set(ControlMode.PercentOutput, 0);
                emptyB = false;
            }
        } else {
            input_.set(ControlMode.PercentOutput,0);
            elevator_.set(ControlMode.PercentOutput, 0);
        }

        // if (Bloaded && !wasBLoaded) { //if a new ball appeared on the bottom
        //     ++balls;
        // }

        if(firing) { //For now the ball reset will be when firing. Will cause issues if you don't fire all balls
            balls = 0;
        } 
        wasTLoaded = Tloaded;
        wasBLoaded = Bloaded;
    }

    public byte getBalls() {
        return balls;
    }

    public void toggleGrabber(double speed) {
        loading = !loading;
        // input_.set(ControlMode.PercentOutput,-speed);
        // elevator_.set(ControlMode.PercentOutput,-speed);
    }

    public void stopGrabber() {
        loading = false;
        inputSpeed = 0;
        liftSpeed = 0;
    }

    public void RunInput(double speed) {
        // if (inputSpeed == 0) {
        //     inputSpeed = -speed;
        // } else {
        //     inputSpeed = 0;
        // }
        input_.set(ControlMode.PercentOutput, -speed);
    }

    public void RunLift(double speed) {
        // if (liftSpeed == 0) {
        //     liftSpeed = -speed;
        // } else {
        //     liftSpeed = 0;
        // }
        elevator_.set(ControlMode.PercentOutput,-speed);
    }

    public boolean emptyTop() { //returns true if there was a ball to empty
        if (isTLoaded()) {
            emptyT = true;
            return true;
        }
        
        return false;
    }

    public boolean emptyBottom() {
        if (isBLoaded()) {
            emptyB = true;
            return true;
        }
        
        return false;
    }

    public boolean isBLoaded() { //bottom sensor loaded
        // return robot_.infraredSensor_.getStatusIsLoaded();
        return true;
    }

    public boolean isTLoaded() { //top sensor loaded
        // return robot_.microSwitch_.getStatusIsLoaded();
        return true;
    }



    public String toString() {
        return "";
    }
}
