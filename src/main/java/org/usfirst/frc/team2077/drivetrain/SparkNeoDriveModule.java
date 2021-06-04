
package org.usfirst.frc.team2077.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.WheelPosition.*;

public class SparkNeoDriveModule extends CANSparkMax implements DriveModuleIF {
    private static final double WHEEL_GEAR_RATIO = 10.714, WHEEL_RADIUS = 4;
    private static final double LAUNCHER_WHEEL_RADIUS = 2, LAUNCHER_GEAR_RATIO = 1;
    private static final int MAX_SHOOTER_RPM = 5400, MAX_WHEEL_RPM = 4500;
    private static final boolean USE_SOFTWARE_PID = true;

    public enum DrivePosition {
        FRONT_RIGHT(NORTH_EAST, 2, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1e-4, 1e-6, 0),
        BACK_RIGHT(SOUTH_EAST, 3, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.1e-4, 1e-6, 0),
        BACK_LEFT(SOUTH_WEST, 4, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.4e-4, 1e-6, 0),
        FRONT_LEFT(NORTH_WEST, 1, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, MAX_WHEEL_RPM, 1.4e-4, 1e-6, 0),

        LEFT_SHOOTER(null, 5, true, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS, MAX_WHEEL_RPM),
        RIGHT_SHOOTER(null, 6, false, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS, MAX_WHEEL_RPM)
        ;

        private final double gearRatio;
        private final double radius;
        private final double maxRPM;
        private final int ID;
        private final boolean INVERSE;
        private final double P, I, D;
        public final WheelPosition WHEEL_POSITION;

        DrivePosition(WheelPosition position, int id, boolean inverse, double gearRatio, double radius, double maxRPM) {
            this(position, id, inverse, gearRatio, radius, maxRPM, 1.4E-4, 1e-6, 0);
        }

        DrivePosition(WheelPosition position, int id, boolean inverse, double gearRatio, double radius, double maxRPM, double p, double i, double d) {
            WHEEL_POSITION = position;
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

    public void setVelocity(final double velocity) {
        //convert from inches/second to rpm
        setPoint = velocity*position.gearRatio*60/circumference;
//        if (setPoint > maxRPM) {
//            setPoint = maxRPM;
//        }
        setRPM(setPoint > maxRPM ? maxRPM : setPoint);
    }

    @Override
    public WheelPosition getWheelPosition() {
        return position.WHEEL_POSITION;
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

    public double getVelocity() {
        final double velocity = encoder.getVelocity()/60/position.gearRatio*circumference; //need to still convert to inches per second
        if (position.INVERSE) {
            return -velocity;
        } else {
            return velocity;
        }
    }

    public double getDistance() {
        return encoder.getPosition()/position.gearRatio*circumference;
    }

    public void resetDistance() {
        encoder.setPosition(0);
    }
}