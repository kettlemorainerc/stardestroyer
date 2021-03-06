
package org.usfirst.frc.team2077.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import static org.usfirst.frc.team2077.Robot.*;

public class SparkNeoDriveModule extends CANSparkMax implements DriveModuleIF {
    public enum DrivePosition {
        FRONT_LEFT(1, false),
        FRONT_RIGHT(2, true), //true
        BACK_LEFT(4, false),
        BACK_RIGHT(3, true) //true
        ;
        public final int ID;
        public final boolean INVERSE;
        DrivePosition(int id, boolean inverse) {
            ID = id;
            INVERSE = inverse;
        }
    }

    //6 inch wheels on rnd bot
    private final CANPIDController pidController;
    private final CANEncoder encoder;
    private double setPoint;
    private final double circumference;
    private final double maxRPM = robot_.constants_.STARDESTROYER_MOTOR_RPM_LIMIT;
    private final double gearRatio = 10.714; // 12.75
    private final boolean isReverse;

    public SparkNeoDriveModule(final DrivePosition pos) {
        this(pos.ID, pos.INVERSE);
    }
    

    public SparkNeoDriveModule(final int deviceID, final boolean isReverse_) {
        super(deviceID, MotorType.kBrushless);
        circumference = robot_.constants_.STARDESTROYER_WHEEL_RADIUS * 2 * Math.PI;
        pidController = this.getPIDController();
        encoder = this.getEncoder();
        isReverse = isReverse_;
        pidController.setP(5e-5);
        pidController.setI(1e-6);
        pidController.setD(0);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-1, 1);


    }
    
    @Override
    public double getMaximumSpeed() {
        return (maxRPM/gearRatio) / (60 / (2 * Math.PI * robot_.constants_.STARDESTROYER_WHEEL_RADIUS));
    }

    /**
     * Set velocity for this wheel.
     * @param velocity In inches/second.
     * Positive values are robot-forward ("north"), negative backward/south.
     */
    public void setVelocity(final double velocity) {
        //convert from inches/second to rpm
        setPoint = velocity*gearRatio*60/circumference;
        if (setPoint > maxRPM) {
            setPoint = maxRPM;
        }
        if (isReverse) {
            pidController.setReference(-setPoint, ControlType.kVelocity);
        } else {
            pidController.setReference(setPoint, ControlType.kVelocity);
        }
    }


    /**
     * Current velocity for this wheel.
     * This should be a direct measurement from an encoder if available,
     * otherwise the set point as passed to {@link #setVelocity} or reported by
     * the motor controller.
     * @return Velocity In inches/second.
     */
    public double getVelocity() {
        final double velocity = encoder.getVelocity()/60/gearRatio*circumference; //need to still convert to inches per second
        if (isReverse == true) {
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
        return encoder.getPosition()/gearRatio*circumference;
    }

    /**
     * Reset the distance measurement to zero inches.
     */
    public void resetDistance() {
        encoder.setPosition(0);
    }
}