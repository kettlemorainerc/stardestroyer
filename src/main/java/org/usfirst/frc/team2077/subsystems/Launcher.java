package org.usfirst.frc.team2077.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2077.drivetrain.SparkNeoDriveModule;
import org.usfirst.frc.team2077.math.ShooterMath;

import static org.usfirst.frc.team2077.Robot.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Launcher extends SubsystemBase implements LauncherIF {

    // motor controllers
    private final TalonSRX screw_;
    private final SparkNeoDriveModule shooterL_;
    private final SparkNeoDriveModule shooterR_;
    private final TalonSRX loader_;


    // elevation screw

    // Readings outside the safe range are assumed to be the result of broken or disconnected hardware.
    // If the screw potentiometer reading is out of range the motor should not be operated.

    private final double powerValue_ = 1; // TODO: Confirm this is a reasonable value or adjust as necessary.

    private final double positionTolerance_ = 4096 * 3; // TODO: Confirm this is a reasonable value or adjust as necessary.
    private final double slowingPoint_ = 4096 * 10;

    // angle setpoint (voltage returned by robot_.potentialSensor_.getScrewVoltage())
    private double screwPosition_ = 0;
    private double lastScrewPosition_ = -1;
    private final double screwEncoderTicks = 4096;
    private double screwDirection_ = 0;
    public boolean zeroing = false;
    private final double maxScrewHeight = 5e6;//7e6; //7934079.0

    // launcher wheels

    public boolean launcherRunning_ = false;
    public double launcherRPM_ = 0;
   
    public final double launcherMaxRPM_ = 4000; // TODO: Put in Constants.
    
    private final double rightLeftBias = 0.0;  // TODO: Put in Constants?  

    private final double unitsToRPM = 1; //(600. / 2048.); //TODO: Fix this
    private final double kP = 0.1; // 0.1
    private final double kI = 0.0001; // 0.0001
    private final double kD = 0.0;

    public ShooterMath shooterMath = new ShooterMath();

    private static String toString(double[] av) {
        return av == null ? "null" : ("" + (Math.round(av[0]*100.)/100.) + "/" + (Math.round(av[1]*10.)/10.));
    }


    public Launcher() {
        screw_ = new TalonSRX(3);
        screw_.configFactoryDefault();

        loader_ = new TalonSRX(2);
        loader_.configFactoryDefault();

        shooterL_ = new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.LEFT_SHOOTER); //0

        shooterR_ = new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.RIGHT_SHOOTER); //1

        launcherRunning_ = false;
        launcherRPM_ = 0;
    }


    @Override
    public void periodic() {
        // stop if position reading is outside normal range due to disconnected wires, etc
        double currentScrewPosition = getScrewPosition();
        shooterMath.setDistance(robot_.crosshairs_.getRange(), currentScrewPosition);
//        launcherRPM_ = (robot_.driveStation_.secondaryStick_.getZ() - 1.) / 2. * -3250 + 2350;
        SmartDashboard.putNumber("hi buddy", robot_.crosshairs_.getRange());


        double error = screwPosition_ - currentScrewPosition;

        if (robot_.microSwitch_.getStatusIsHit() && screwDirection_ < 0) {
            screw_.set(ControlMode.PercentOutput, 0);
            zeroScrew();
            return;
        }

        // stop if close enough to the set point
        if (Math.abs(error) < positionTolerance_) {
            screw_.set(ControlMode.PercentOutput, 0);
            screwDirection_ = 0;
            return;
        }

        // stop if past the set point in the last adjustment direction
        if (Math.signum(error) != screwDirection_) {
            screw_.set(ControlMode.PercentOutput, 0);
            screwDirection_ = 0;
            return;
        }

        // TODO: could vary motor with error magnitude
//        screw_.set(ControlMode.PercentOutput, powerValue_ * screwDirection_);

        //slow if close enough to the set point
        if (Math.abs(error) < slowingPoint_) {
            screw_.set(ControlMode.PercentOutput, .4 * screwDirection_);
            return;
        }

        screw_.set(ControlMode.PercentOutput, screwDirection_);
    }



    @Override
    public boolean setRangeLower(double range) {
        return false;
    }

    @Override
    public boolean setRangeUpper(double range) {
        launcherRPM_ = shooterMath.getRangeAV()[1];
        SmartDashboard.putNumber("CalculatedScrew: ", this.shooterMath.getNeededAngle(false));
        SmartDashboard.putNumber("CalculatedTicks: ", this.shooterMath.getNeededAngle((true)));
        this.setScrewPosition(this.shooterMath.getNeededAngle(true), true);

//        double[] av = getAngleVelocity(range);

//        setScrewPosition(av[0]);
//        setLauncherRPM(av[1]);

        return true;
    }

    @Override
    public void setRunning(boolean running) {
        launcherRunning_ = running;
        runLauncher(launcherRunning_ ? launcherRPM_ : 0);
    }

    @Override
    public boolean isRunning() {
        return launcherRunning_;
    }

    @Override
    public boolean isReady() {
        // TODO: isRunning() && isLoaded() && both motors within tolerance of target && angle withing tolerance of target voltage
        double wantedMotorSpeed = launcherRPM_;
        return Math.abs(wantedMotorSpeed - getLauncherSpeed()[0]) < 50 && Math.abs(wantedMotorSpeed - getLauncherSpeed()[1]) < 50 && screwPosition_ - getScrewPosition() == 0;
//        return Math.abs(screwPosition_ - getScrewPosition()) <= 0.1;
    }

    @Override
    public void load() {
        // if (!isLoaded()) {
        //     loader_.set(ControlMode.PercentOutput,1);
        // } else {
        //     loader_.set(ControlMode.PercentOutput,0);
        // }
    }

    @Override
    public boolean isLoaded() {
        //return robot_.microSwitch_.getStatusIsLoaded();
        if (robot_.tgrabber_.getBalls() > 0) {
            return true;
        }
        return false;
        //return robot_.infraredSensor_.getStatusIsLoaded();
    }

    @Override
    public void launch() {
        // if (isLoaded()) {
        setRangeUpper(robot_.crosshairs_.getRange());
        setRunning(true);
//        if (isReady()) {
//            runLoader(0.6);
//
//        }
    }

    public void stopLaunch() {
        setRunning(false);
        runLoader(0);
    }





    public void runAngle(boolean dir) {
        if(dir) {
            screw_.set(ControlMode.PercentOutput, 1.0);
        } else {
            screw_.set(ControlMode.PercentOutput, -1.0);
        }
        
    }

    public void stopAngle() {
        screw_.set(ControlMode.PercentOutput, 0.0);
    }







    private void runLauncher(double velocity) {
        double leftRPM = 0;
        double rightRPM = 0;
        if (velocity != 0) {
            double bias = velocity * rightLeftBias / 2.;
            
            leftRPM = (velocity - bias) / unitsToRPM;
            rightRPM = (velocity + bias) / unitsToRPM;
        }
        shooterL_.setRPM(leftRPM);
        shooterR_.setRPM(rightRPM);
    }

    @Deprecated
    public double getLaunchVelL() {
        double res = shooterL_.getRPM() * unitsToRPM;

        
        return res;
    }
    @Deprecated
    public double getLaunchVelR() {
        return shooterR_.getRPM() * unitsToRPM;
    }

    public double[] getLauncherSpeed() {
        return new double[] {shooterL_.getRPM() * unitsToRPM, shooterR_.getRPM() * unitsToRPM};
    }




    public void runLoader(double speed) {
        loader_.set(ControlMode.PercentOutput,-speed);
    }

    public void stopLoader() {
        loader_.set(ControlMode.PercentOutput,0.0);
    }
   
    public void stopAll() {
        shooterL_.setVelocity(0.0);
        shooterR_.setVelocity(0.0);
        loader_.set(ControlMode.PercentOutput,0.0);
        screw_.set(ControlMode.PercentOutput,0.0);
    }

    public double getScrewPosition() {
        return screw_.getSelectedSensorPosition();
    }

    public void zeroScrew() {
        screw_.setSelectedSensorPosition(0);
        screwPosition_ = 0;
        screwDirection_ = 0;
        zeroing = false;
        return;
    }


    public void setScrewPosition(double distance, boolean isHeight){ //NEEDS CHECK
        double currentScrewPosition = getScrewPosition();

        if (isHeight) {
            if (screwPosition_ >= maxScrewHeight && screwDirection_ == 1) return;
            screwPosition_ = Math.max(-maxScrewHeight, Math.min(distance, maxScrewHeight));
            screwDirection_ = Math.signum(screwPosition_ - currentScrewPosition);
            return;
        }
        if (screwPosition_ >= maxScrewHeight && screwDirection_ == 1) return;
        screwPosition_ = Math.max(-maxScrewHeight, Math.min(distance + getScrewPosition(), maxScrewHeight));
        screwDirection_ = Math.signum(screwPosition_ - currentScrewPosition);
    }

    public void setScrewPosition01(double setpoint01) {

    }

    public void setLauncherRPM(double rpm) {
        launcherRPM_ = rpm;
        if (launcherRunning_) {
            runLauncher(launcherRPM_);
        }
    }

    public double[] getRangeAV() {
        return new double[] {this.shooterMath.getRangeAV()[0], this.launcherRPM_};
//        return this.shooterMath.getRangeAV();
    }
}
