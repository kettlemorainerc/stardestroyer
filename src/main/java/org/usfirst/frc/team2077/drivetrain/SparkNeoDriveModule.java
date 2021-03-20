
package org.usfirst.frc.team2077.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import static org.usfirst.frc.team2077.Robot.*;

public class SparkNeoDriveModule extends CANSparkMax implements DriveModuleIF {
    private static final double WHEEL_GEAR_RATIO = 10.714;
    private static final double WHEEL_RADIUS = 4;
    private static final double LAUNCHER_WHEEL_RADIUS = 2;
    private static final double LAUNCHER_GEAR_RATIO = 1;
    private static final double ORIGINAL_P = 5e-5;
    private static final double ORIGINAL_I = 1e-6;
    private static final double ORIGINAL_D = 0;
    private static final boolean USE_ORIGINAL_PID = false;

    public enum DrivePosition {
        FRONT_RIGHT(2, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS),
        BACK_RIGHT(3, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS),
        BACK_LEFT(4, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS),
        FRONT_LEFT(1, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS),

        LEFT_SHOOTER(5, true, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS),
        RIGHT_SHOOTER(6, false, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS)
        ;
        private final double gearRatio;
        private final double radius;
        public final int ID;
        public final boolean INVERSE;
        DrivePosition(int id, boolean inverse, double gearRatio, double radius) {
            ID = id;
            INVERSE = inverse;
            this.gearRatio = gearRatio;
            this.radius = radius;
        }
    }

    //6 inch wheels on rnd bot
    private final CANPIDController pidController;
    private final CANEncoder encoder;
    private double setPoint;
    private final double circumference;
    private final double maxRPM = robot_.constants_.STARDESTROYER_MOTOR_RPM_LIMIT;
    private final DrivePosition position;

    public SparkNeoDriveModule(final DrivePosition pos) {
        super(pos.ID, MotorType.kBrushless);
        this.position = pos;
        circumference = pos.radius * 2 * Math.PI;
        pidController = this.getPIDController();
        encoder = this.getEncoder();

        if(USE_ORIGINAL_PID) {
            pidController.setP(ORIGINAL_P);
            pidController.setI(ORIGINAL_I);
            pidController.setD(ORIGINAL_D);
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