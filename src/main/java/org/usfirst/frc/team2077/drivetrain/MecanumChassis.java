/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.math.AccelerationLimits;

import java.util.*;

import static org.usfirst.frc.team2077.Robot.robot_;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction.*;
import static org.usfirst.frc.team2077.math.AccelerationLimits.Type.*;

public class MecanumChassis extends AbstractChassis {
	private static final double WHEELBASE = 20.375; // inches
	private static final double TRACK_WIDTH = 22.625; // inches
	private static final double WHEEL_RADIUS = 4.0; // inches
	private static final double EAST_ADJUSTMENT = .65;

	private static double minSpeedFromMax(double maxSpeed) {
		// TODO: Test and configure.
		return maxSpeed * .1;
	}

	private final MecanumMath mecanumMath_;

	private static DriveModuleIF[] buildDriveModule() {
		return new DriveModuleIF[]{
			new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.FRONT_RIGHT),  // northeast (right front)
			new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.BACK_RIGHT),  // southeast (right rear)
			new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.BACK_LEFT),  // southwest (left rear)
			new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.FRONT_LEFT)   // northwest (left front)
		};
	}

	public MecanumChassis() {
		super(buildDriveModule());

		mecanumMath_ = new MecanumMath(WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, WHEEL_RADIUS, 1, 180 / Math.PI);

		// north/south speed conversion from 0-1 range to DriveModule maximum (inches/second)
		maximumSpeed_ = driveModule_.values()
									.stream()
									.mapToDouble(DriveModuleIF::getMaximumSpeed)
									.min()
									.orElseThrow();
		// rotation speed conversion from 0-1 range to DriveModule maximum (degrees/second)
		EnumMap<WheelPosition, Double> maxRotation = new EnumMap<>(WheelPosition.class);
		maxRotation.put(WheelPosition.NORTH_EAST, -maximumSpeed_);
		maxRotation.put(WheelPosition.SOUTH_EAST, -maximumSpeed_);
		maxRotation.put(WheelPosition.NORTH_WEST, -maximumSpeed_);
		maxRotation.put(WheelPosition.SOUTH_WEST, -maximumSpeed_);

		maximumRotation_ = mecanumMath_.forward(maxRotation).get(CLOCKWISE);

		// lowest chassis speeds supportable by the drive modules
		minimumSpeed_ = minSpeedFromMax(maximumSpeed_);
		minimumRotation_ = minSpeedFromMax(maximumRotation_);

		// If you're interested in the specifics behind formatting you should look into https://docs.oracle.com/javase/8/docs/api/java/util/Formatter.html#syntax
		System.out.printf("%s MAXIMUM [SPEED / ROTATION]: [%.1f IN/SEC / %.1f DEG/SEC]%n",
		                  getClass().getName(),
		                  maximumSpeed_,
		                  maximumRotation_);
		System.out.printf("%s MINIMUM [SPEED / ROTATION]: [%.1f IN/SEC / %.1f DEG/SEC]%n",
		                  getClass().getName(),
		                  minimumSpeed_,
		                  minimumRotation_);

		double[][] a = getAccelerationLimits();
		System.out.printf("%s ACCELERATION LIMITS: [DIRECTION MAX/MIN][NORTH %.1f/%.1f][EAST %.1f/%.1f][ROTATION %.1f/%.1f]%n",
		                  getClass().getName(),
		                  accelerationLimits.get(NORTH, ACCELERATION),
		                  accelerationLimits.get(NORTH, DECELERATION),
		                  accelerationLimits.get(EAST, ACCELERATION),
		                  accelerationLimits.get(EAST, DECELERATION),
		                  accelerationLimits.get(CLOCKWISE, ACCELERATION),
		                  accelerationLimits.get(CLOCKWISE, DECELERATION));
	}

	@Override
	public void setVelocity(double north, double east, double clockwise, AccelerationLimits accelerationLimits) {
		setVelocity(north, east, accelerationLimits);
		setRotation(clockwise, accelerationLimits);
	}

	@Override
	public void setVelocity(double north, double east, AccelerationLimits accelerationLimits) {
		setVelocity.put(NORTH, north);
		northSet_ = north;

		accelerationLimits.set(NORTH, accelerationLimits.get(NORTH));
		northAccelerationLimit_ = accelerationLimits.get(NORTH);

		setVelocity.put(EAST, east);
		eastSet_ = east;

		accelerationLimits.set(EAST, accelerationLimits.get(EAST));
		eastAccelerationLimit_ = accelerationLimits.get(EAST);
	}

	@Override
	public void setRotation(double clockwise, AccelerationLimits accelerationLimits) {
		setVelocity.put(CLOCKWISE, clockwise);
		clockwiseSet_ = clockwise;

		accelerationLimits.set(CLOCKWISE, accelerationLimits.get(CLOCKWISE));
		rotationAccelerationLimit_ = accelerationLimits.get(CLOCKWISE);
	}

	@Override
	public EnumMap<Direction, Double> getVelocitySet() {
		return setVelocity;
//		return new double[]{northSet_, eastSet_, clockwiseSet_};
	}

	@Override
	public EnumMap<Direction, Double> getVelocityCalculated() {
		return calculatedVelocity;
	}

	public EnumMap<Direction, Double> getCalculatedVelocity() {
		return calculatedVelocity;
	}

	@Override
	public EnumMap<Direction, Double> getVelocityMeasured() {
		return velocityMeasured_;
	}

	@Override
	protected void updatePosition() {

		// chassis velocity from internal set point
//		velocitySet_ = getVelocityCalculated();
		// chassis velocity from motor/wheel measurements
		EnumMap<WheelPosition, Double> wheelVelocities = new EnumMap<>(WheelPosition.class);
//		double[] wheelVelocities = new double[AssemblyPosition.values().length];
		driveModule_.forEach((k, wheel) -> wheelVelocities.put(wheel.getWheelPosition(), wheel.getVelocity()));
		velocityMeasured_ = mecanumMath_.forward(wheelVelocities);

		// TODO: E/W velocities are consistently lower than those calculated from wheel speeds.
		// TODO: Measure actual vs measured E/W distances and insert an adjustment factor here.
		// TODO: Put the adjustment factor in constants.
		setVelocity.compute(EAST, (k, v) -> v * EAST_ADJUSTMENT);
//		velocitySet_[VelocityDirection.EAST.ordinal()] *= EAST_ADJUSTMENT; // just a guess

		velocityMeasured_.compute(EAST, (k, val) -> val * EAST_ADJUSTMENT); // just a guess

		// update position with motion since last update
		positionSet_.moveRelative(
			setVelocity.get(NORTH) * timeSinceLastUpdate_,
//			velocitySet_[NORTH.ordinal()] * timeSinceLastUpdate_,
			setVelocity.get(EAST) * timeSinceLastUpdate_,
//			velocitySet_[VelocityDirection.EAST.ordinal()] * timeSinceLastUpdate_,
			setVelocity.get(CLOCKWISE) * timeSinceLastUpdate_
//			velocitySet_[VelocityDirection.CLOCKWISE.ordinal()] * timeSinceLastUpdate_
		);
		positionMeasured_.moveRelative(
			velocityMeasured_.get(NORTH) * timeSinceLastUpdate_,
			velocityMeasured_.get(EAST) * timeSinceLastUpdate_,
			velocityMeasured_.get(CLOCKWISE) * timeSinceLastUpdate_
		);
		if(robot_.angleSensor_ != null) { // TODO: Confirm AngleSensor is actually reading. Handle bench testing.
			double[] pS = positionSet_.get();
			double[] pM = positionMeasured_.get();
			pS[CLOCKWISE.ordinal()] = pM[CLOCKWISE.ordinal()] = robot_.angleSensor_.getAngle(); // TODO: conditional on gyro availability
			positionSet_.set(pS[NORTH.ordinal()],
			                 pS[EAST.ordinal()],
			                 pS[CLOCKWISE.ordinal()]);

			positionMeasured_.set(pM[NORTH.ordinal()],
			                      pM[EAST.ordinal()],
			                      pM[CLOCKWISE.ordinal()]);
		}
	}

	@Override
	protected void updateDriveModules() {

//		double[] v = getVelocityCalculated();
		//    if (debug_ ) System.out.print("CHASSIS: " + Math.round(v[0]*10.)/10. + " " + Math.round(v[1]*10.)/10. + " " + Math.round(v[2]*10.)/10.);
//		double[] w = mecanumMath_.inverse(v);

		// compute motor speeds
		EnumMap<WheelPosition, Double> wheelSpeed = mecanumMath_.inverse(calculatedVelocity);

		double max = wheelSpeed.values()
		                       .stream()
		                       .mapToDouble(a -> a)
		                       .max()
		                       .orElseThrow();

		driveModule_.forEach((position, wheel) -> wheel.setVelocity(wheelSpeed.get(wheel.getWheelPosition()) / max));

		// scale all motors proportionally if any are out of range
//		double max = 1;
//		for(double ws : w) {
//			max = Math.max(max, Math.abs(ws) / maximumSpeed_);
//		}

		//    if (debug_ ) System.out.print(" WHEELS:");
//		for(int i = 0; i < w.length; i++) {
//			double ws = w[i] / max;
//			driveModule_[i].setVelocity(ws);
//			//        if (debug_ ) System.out.print(" " + Math.round(100.*ws)/100. + "(" + Math.round(100.*driveModule_[i].getVelocity())/100. + ")");
//		}
		// if (debug_ ) System.out.println(" " + this);
	}

	@Override
	public String toString() {
		return String.format("[Velocity: %s][Wheels: %s]",
							 calculatedVelocity,
							 driveModule_);
	}
}
