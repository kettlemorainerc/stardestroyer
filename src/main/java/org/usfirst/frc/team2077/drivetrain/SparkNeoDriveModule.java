
package org.usfirst.frc.team2077.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team2077.Robot.*;

public class SparkNeoDriveModule extends CANSparkMax implements DriveModuleIF {
    private static final double WHEEL_GEAR_RATIO = 10.714, WHEEL_RADIUS = 4;
    private static final double LAUNCHER_WHEEL_RADIUS = 2, LAUNCHER_GEAR_RATIO = 1;
    private static final int MAX_SHOOTER_RPM = 5400, MAX_WHEEL_RPM = 3000;
    private static final boolean USE_SOFTWARE_PID = true;

    public enum DrivePosition {
        FRONT_RIGHT(2, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1e-4, 1e-6, 0),
        BACK_RIGHT(3, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.1e-4, 1e-6, 0),
        BACK_LEFT(4, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.4e-4, 1e-6, 0),
        FRONT_LEFT(1, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.4e-4, 1e-6, 0),

        LEFT_SHOOTER(5, true, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS, MAX_SHOOTER_RPM),
        RIGHT_SHOOTER(6, false, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS, MAX_SHOOTER_RPM)
        ;
        private final double gearRatio;
        private final double radius;
        private final double maxRPM;
        public final int ID;
        public final boolean INVERSE;
        public final double P, I, D;
        DrivePosition(int id, boolean inverse, double gearRatio, double radius, double maxRPM) {
            this(id, inverse, gearRatio, radius, maxRPM, 1.4e-4, 1e-6, 0);
        }

        DrivePosition(int id, boolean inverse, double gearRatio, double radius, double maxRPM, double p, double i, double d) {
            ID = id;
            INVERSE = inverse;
            this.gearRatio = gearRatio;
            this.radius = radius;
            this.maxRPM = maxRPM;
            this.P = p;
            this.I = i;
            this.D = d;
        }

    }

    //6 inch wheels on rnd bot
    private final CANPIDController pidController;
    private final CANEncoder encoder;
    private double setPoint;
    private final double circumference;
    private final double maxRPM;
    private final DrivePosition position;

    public SparkNeoDriveModule(final DrivePosition pos) {
        super(pos.ID, MotorType.kBrushless);
        this.position = pos;
        maxRPM = pos.maxRPM;
        circumference = pos.radius * 2 * Math.PI;
        pidController = this.getPIDController();
        encoder = this.getEncoder();

        if(USE_SOFTWARE_PID || position == DrivePosition.LEFT_SHOOTER || position == DrivePosition.RIGHT_SHOOTER) {
            pidController.setP(position.P);
            pidController.setI(position.I);
            pidController.setD(position.D);
            pidController.setIZone(0);
            pidController.setFF(0);
            pidController.setOutputRange(-1, 1);
        }

    }

    public DrivePosition getPosition() {
        return position;
    }
    
    @Override
    public double getMaximumSpeed() {
        return (maxRPM/position.gearRatio) / (60 / (2 * Math.PI * position.radius));
    }

    /**
     * Set velocity for this wheel.
     * @param velocity In inches/second.
     * Positive values are robot-forward ("north"), negative backward/south.
     */
    public void setVelocity(final double velocity) {
        //convert from inches/second to rpm
        setPoint = velocity*position.gearRatio*60/circumference;
        if (setPoint > maxRPM) {
            setPoint = maxRPM;
        }
        setRPM(setPoint);
    }

    public void setRPM(double rpm) {
        setPoint = Math.min(rpm, maxRPM);
        if (position.INVERSE) {
            pidController.setReference(-setPoint, ControlType.kVelocity);
        } else {
            pidController.setReference(setPoint, ControlType.kVelocity);
        }
    }

    public double getRPM() {
        final double velocity = encoder.getVelocity();
        if (position.INVERSE) {
            return -velocity;
        } else {
            return velocity;
        }
    }

    public double getSetPoint() {
        if (position.INVERSE) {
            return -setPoint;
        }
        return setPoint;
    }

    /**
     * Current velocity for this wheel.
     * This should be a direct measurement from an encoder if available,
     * otherwise the set point as passed to {@link #setVelocity} or reported by
     * the motor controller.
     * @return Velocity In inches/second.
     */
    public double getVelocity() {
        final double velocity = encoder.getVelocity()/60/position.gearRatio*circumference; //need to still convert to inches per second
        if (position.INVERSE) {
            return -velocity;
        } else {
            return velocity;
        }
    }


    /**
     * Distance traveled by the wheel since startup or the last reset.
     * This should be a direct measurement from an encoder if available,
     * otherwise computed by integrating velocity over time.
     * @return Distance in inches.
     */
    public double getDistance() {
        return encoder.getPosition()/position.gearRatio*circumference;
    }

    /**
     * Reset the distance measurement to zero inches.
     */
    public void resetDistance() {
        encoder.setPosition(0);
    }
}