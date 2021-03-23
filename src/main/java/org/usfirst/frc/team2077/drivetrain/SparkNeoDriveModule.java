
package org.usfirst.frc.team2077.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

import static org.usfirst.frc.team2077.Robot.*;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.AssemblyPosition.*;

public class SparkNeoDriveModule extends CANSparkMax implements DriveModuleIF {
    private static final double WHEEL_GEAR_RATIO = 10.714, WHEEL_RADIUS = 4;
    private static final double LAUNCHER_WHEEL_RADIUS = 2, LAUNCHER_GEAR_RATIO = 1;
    private static final double ORIGINAL_P = 5e-5, ORIGINAL_I = 1e-6, ORIGINAL_D = 0;
    private static final boolean USE_SOFTWARE_PID = false, USE_ORIGINAL_PID = false;

    public enum DrivePosition {
        FRONT_RIGHT(NORTH_EAST, 2, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, 1e-4, 1e-6, 0),
        BACK_RIGHT(SOUTH_EAST, 3, true, WHEEL_GEAR_RATIO, WHEEL_RADIUS, 1.1e-4, 1e-6, 0),
        BACK_LEFT(SOUTH_WEST, 4, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, 1.4e-4, 1e-6, 0),
        FRONT_LEFT(NORTH_WEST, 1, false, WHEEL_GEAR_RATIO, WHEEL_RADIUS, 1.4e-4, 1e-6, 0),

        LEFT_SHOOTER(null, 5, true, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS),
        RIGHT_SHOOTER(null, 6, false, LAUNCHER_GEAR_RATIO, LAUNCHER_WHEEL_RADIUS)
        ;

        private final double gearRatio;
        private final double radius;
        private final int ID;
        private final boolean INVERSE;
        private final double P, I, D;
        private final AssemblyPosition WHEEL_POSITION;

        DrivePosition(AssemblyPosition position, int id, boolean inverse, double gearRatio, double radius) {
            this(position, id, inverse, gearRatio, radius, ORIGINAL_P, ORIGINAL_I, ORIGINAL_D);
        }

        DrivePosition(AssemblyPosition position, int id, boolean inverse, double gearRatio, double radius, double p, double i, double d) {
            WHEEL_POSITION = position;
            ID = id;
            INVERSE = inverse;
            this.gearRatio = gearRatio;
            this.radius = radius;
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
    private final double maxRPM = robot_.constants_.STARDESTROYER_MOTOR_RPM_LIMIT;
    private final DrivePosition position;

    public SparkNeoDriveModule(final DrivePosition pos) {
        super(pos.ID, MotorType.kBrushless);
        this.position = pos;
        circumference = pos.radius * 2 * Math.PI;
        pidController = this.getPIDController();
        encoder = this.getEncoder();

        if(USE_SOFTWARE_PID) {
            if(USE_ORIGINAL_PID) {
                pidController.setP(ORIGINAL_P);
                pidController.setI(ORIGINAL_I);
                pidController.setD(ORIGINAL_D);
            } else {
                pidController.setP(position.P);
                pidController.setI(position.I);
                pidController.setD(position.D);
            }
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
        if (setPoint > maxRPM) {
            setPoint = maxRPM;
        }
        setRPM(setPoint);
    }

    @Override
    public AssemblyPosition getWheelPosition() {
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