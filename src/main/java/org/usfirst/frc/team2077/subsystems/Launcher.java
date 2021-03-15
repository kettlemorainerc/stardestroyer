package org.usfirst.frc.team2077.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team2077.drivetrain.SparkNeoDriveModule;

import static org.usfirst.frc.team2077.Robot.*;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Launcher extends SubsystemBase implements LauncherIF {
    private static final double LAUNCHER_WHEEL_RADIUS = 2;

    // motor controllers
    private final TalonSRX screw_;
    private final SparkNeoDriveModule shooterL_;
    private final SparkNeoDriveModule shooterR_;
    private final TalonSRX loader_;


    // elevation screw

    // Readings outside the safe range are assumed to be the result of broken or disconnected hardware.
    // If the screw potentiometer reading is out of range the motor should not be operated.
    private final double[] safeVoltageRange_ = {1.4, 5.1}; // TODO: Confirm this is a reasonable range or adjust as necessary.

    private final double[] operatingVoltageRange_ = {2.2, 4.9};

    private final double powerValue_ = 1; // TODO: Confirm this is a reasonable value or adjust as necessary.

    private final double positionTolerance_ = .02; // TODO: Confirm this is a reasonable value or adjust as necessary.

    // angle setpoint (voltage returned by robot_.potentialSensor_.getScrewVoltage())
    private double screwPosition_ = 1; // TODO: this should reflect hardware position at startup.
    private double screwDirection_ = 0.0; // setpoint changes operate in one direction only to avoid oscillation/instability


    // launcher wheels

    public boolean launcherRunning_ = false;
    public double launcherRPM_ = 0;
   
    public final double launcherMaxRPM_ = 4000; // TODO: Put in Constants.
    
    private final double rightLeftBias = 0.0;  // TODO: Put in Constants?  

    private final double unitsToRPM = 1; //(600. / 2048.); //TODO: Fix this
    private final double kP = 0.1; // 0.1
    private final double kI = 0.0001; // 0.0001
    private final double kD = 0.0;

    
    private static double[] a_ = {0.1, 0.6, 1.3, 1.9, 2.6}; // measured calibration points
    private static double[] v_ = {3000, 4000, 5000, 6000}; // measured calibration points
    private static double[][] rangeAV_ = { // ranges in feet with pseudo-index values into a_ and v_
        {7.5,  5,   4},
        {8.5,  5,   4},
        {9,    4,   3.5},
        {9.5,  3,   3},
        {12,   2,   3},
        {15,   1.5, 2},
        {17.5, 1,   2},
        {20,   1.5, 2},
        {28,   2,   3},
        {32,   2,   4},
        {38,   5,   4}
    };
    private static double[] angle_; // enlarged table with n-1 interpolated values
    private static double[] velocity_; // enlarged table with n-1 interpolated values
    private static double[] range_; // rangeAV_ range values as a searchable array
    static {
        angle_ = new double[a_.length*2-1];
        for (int i = 0; i < angle_.length; i+=2) {
            angle_[i] = a_[i/2];
        }
        for (int i = 1; i < angle_.length; i+=2) {
            angle_[i] = (angle_[i-1] + angle_[i+1]) / 2.;
        }
        velocity_ = new double[v_.length*2-1];
        for (int i = 0; i < velocity_.length; i+=2) {
            velocity_[i] = v_[i/2];
        }
        for (int i = 1; i < velocity_.length; i+=2) {
            velocity_[i] = (velocity_[i-1] + velocity_[i+1]) / 2.;
        }
        range_ = new double[rangeAV_.length];
        for (int i = 0; i < range_.length; i++) {
            range_[i] = 12. * rangeAV_[i][0]; // in inches
        }
    }

    
    private static double[] getAngleVelocity(double range) {
        if (range < range_[0] || range >= range_[range_.length-1]) {
            return null;
        }
        int i = 0;
        while (range > range_[i]) i++;
        int[] ravIndex = {i-1, i};
        double[] r = {range_[ravIndex[0]], range_[ravIndex[1]]};
        double[] a = {angle_[(int)Math.round((rangeAV_[ravIndex[0]][1]-1.)*2.)], angle_[(int)Math.round((rangeAV_[ravIndex[1]][1]-1.)*2.)]};
        double[] v = {velocity_[(int)Math.round((rangeAV_[ravIndex[0]][2]-1.)*2.)], velocity_[(int)Math.round((rangeAV_[ravIndex[1]][2]-1.)*2.)]};
        double interpolation = (range - r[0]) / (r[1] - r[0]);
        
        double angle = a[0] + interpolation * (a[1] - a[0]);
        double velocity = v[0] + interpolation * (v[1] - v[0]);
        return new double[] {angle, velocity};
    }

    /**
     * Test code for  verifying range/angle/velocity tables. May be run standalone from VSCode.
     */
    public static void main(String[] argv) {
        for (double r : range_) System.out.println("R:" + r/12.);
        for (double a : angle_) System.out.println("A:" + a);
        for (double v : velocity_) System.out.println("V:" + v);
        System.out.println();
        for (int i = 6; i <= 40; i++) {
            System.out.println("" + i + " " + toString(getAngleVelocity(i * 12.)));
        }
    }

    private static String toString(double[] av) {
        return av == null ? "null" : ("" + (Math.round(av[0]*100.)/100.) + "/" + (Math.round(av[1]*10.)/10.));
    }


    public Launcher() {
        screw_ = new TalonSRX(3);
        screw_.configFactoryDefault();

        loader_ = new TalonSRX(2);
        loader_.configFactoryDefault();

        shooterL_ = new SparkNeoDriveModule(5, true, 1, LAUNCHER_WHEEL_RADIUS); //0
        // shooterL_.configFactoryDefault();
        // shooterL_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
        // shooterL_.setSensorPhase(true);
        // shooterL_.configNominalOutputForward(0, 0);
		// shooterL_.configNominalOutputReverse(0, 0);
		// shooterL_.configPeakOutputForward(1, 0);
        // shooterL_.configPeakOutputReverse(-1, 0);
        // shooterL_.config_kF(0, 0.0, 0);
		// shooterL_.config_kP(0, kP, 0);
		// shooterL_.config_kI(0, kI, 0);
		// shooterL_.config_kD(0, kD, 0);

        shooterR_ = new SparkNeoDriveModule(6, false, 1, LAUNCHER_WHEEL_RADIUS); //1
        // shooterR_.configFactoryDefault();
        // shooterR_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
        // shooterR_.setSensorPhase(true);
        // shooterR_.configNominalOutputForward(0, 0);
		// shooterR_.configNominalOutputReverse(0, 0);
		// shooterR_.configPeakOutputForward(1, 0);
		// shooterR_.configPeakOutputReverse(-1, 0);
        // shooterR_.config_kF(0, 0.0, 0);
		// shooterR_.config_kP(0, kP, 0);
		// shooterR_.config_kI(0, kI, 0);
		// shooterR_.config_kD(0, kD, 0);

        launcherRunning_ = false;
        launcherRPM_ = 0;
    }


    @Override
    public void periodic() { // continuous adjustment of screw motor toward setpoint

        // stop if position reading is outside normal range due to disconnected wires, etc
        double currentScrewPosition = getScrewPosition();
        SmartDashboard.putNumber("hi buddy", currentScrewPosition);
        if ((currentScrewPosition < safeVoltageRange_[0]) || (currentScrewPosition > safeVoltageRange_[1])) {
            screw_.set(ControlMode.PercentOutput, 0);
            return;
        }
        // stop if close enough to the set point
        double error = screwPosition_ - currentScrewPosition;
        if (Math.abs(error) < positionTolerance_) {
            screw_.set(ControlMode.PercentOutput, 0);
            return;
        }

        // stop if past the set point in the last adjustment direction
        if (Math.signum(error) != screwDirection_) {
            screw_.set(ControlMode.PercentOutput, 0);
            return;
        }
        
        // TODO: could vary motor with error magnitude
        screw_.set(ControlMode.PercentOutput, -1 * powerValue_ * screwDirection_);
    }



    @Override
    public boolean setRangeLower(double range) {
        return false;
    }

    @Override
    public boolean setRangeUpper(double range) {

        double[] av = getAngleVelocity(range);
        if (av == null) {
            return false;
        }

        setScrewPosition(av[0]);
        setLauncherRPM(av[1]);

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
        //return Math.abs(wantedMotorSpeed - getLauncherSpeed()[0]) < 50 && Math.abs(wantedMotorSpeed - getLauncherSpeed()[1]) < 50 && screwPosition_ - getScrewPosition() == 0;
        return Math.abs(screwPosition_ - getScrewPosition()) <= 0.1;
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
        if (isReady()) {
            runLoader(1.0);
            
        }
        //}  else {
        //     load(); //TODO: Change for new ir-sensor/micro-switch
        // }
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
        return robot_.potentialSensor_.getScrewVoltage();
    }


    public void setScrewPosition(double goalVoltage){ //NEEDS CHECK

        // Goal voltage within paramaters check
        goalVoltage = Math.max(goalVoltage, operatingVoltageRange_[0]);
        goalVoltage = Math.min(goalVoltage, operatingVoltageRange_[1]);
        SmartDashboard.putNumber("testing?", goalVoltage);
        screwPosition_ = goalVoltage;
        screwDirection_ = Math.signum(goalVoltage - getScrewPosition()); // 1.0 if to be adjusted up, -1.0 if down
    }

    public void setScrewPosition01(double setpoint01) {
        setScrewPosition(operatingVoltageRange_[0] + (operatingVoltageRange_[1]-operatingVoltageRange_[0]) * setpoint01);
    }

    public void setLauncherRPM(double rpm) {
        launcherRPM_ = rpm;
        if (launcherRunning_) {
            runLauncher(launcherRPM_);
        }
    }
}
