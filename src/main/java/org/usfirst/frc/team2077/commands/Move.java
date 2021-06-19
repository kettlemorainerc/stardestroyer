/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection;
import org.usfirst.frc.team2077.math.AccelerationLimits;
import org.usfirst.frc.team2077.math.Position;

import java.util.EnumMap;

import static java.lang.Math.*;
import static org.usfirst.frc.team2077.Robot.robot_;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;
import static org.usfirst.frc.team2077.math.AccelerationLimits.Type.*;


public class Move extends CommandBase {
	 public final double ACCELERATION_G_LIMIT = .1;
	 public final double DECELERATION_G_LIMIT = .3;

	public enum Style {
		MOVE_AND_ROTATE,
		MOVE,
		ROTATE
	}
	
	public static <T> EnumMap<VelocityDirection, T> directionMap() {
		return new EnumMap<>(VelocityDirection.class);
	}

	private final EnumMap<VelocityDirection, Double> targetDistance;
	private EnumMap<VelocityDirection, Double> vCurrent = directionMap();
	private EnumMap<VelocityDirection, Double> fast_ = directionMap();
	private EnumMap<VelocityDirection, Double> slow_ = directionMap();
	private EnumMap<VelocityDirection, Double> distanceRemaining_ = directionMap();
	private EnumMap<VelocityDirection, Boolean> finished_ = directionMap();
	private AccelerationLimits acceleration_;
	private final Style method_;
	private Position origin_;

	public Move(double north, double east, double rotation) {
		this(north, east, rotation, Style.MOVE_AND_ROTATE, robot_.position_, robot_.heading_);
	}

	public Move(double north, double east) {
		this(north, east, 0, Style.MOVE, robot_.position_);
	}

	public Move(double rotation) {
		this(0, 0, rotation, Style.ROTATE, robot_.heading_);
	}

	private Move(double north, double east, double rotation, Style method, Subsystem... requirements) {

		addRequirements(requirements);
		// distanceTotal_ = new double[] {north, east * .68, rotation * 7/8}; //fudged values for the multipliers
		targetDistance = new EnumMap<>(VelocityDirection.class);
		targetDistance.put(NORTH, north);
		targetDistance.put(EAST, east * .68);
		targetDistance.put(ROTATION, rotation * 7/8);
		method_ = method;
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 DISTANCE:" +
						   targetDistance.get(NORTH) +
						   " " +
						   targetDistance.get(EAST) +
						   " " +
						   targetDistance.get(ROTATION) +
						   " (" +
						   method_ +
						   ")");
	}


	@Override
	public void initialize() {
		EnumMap<VelocityDirection, Double> max = robot_.chassis_.getMaximumVelocity();
		EnumMap<VelocityDirection, Double> min = robot_.chassis_.getMinimumVelocity();

		// scale factors for north/east/rotation by fraction of maximum velocity
		double[] scale = {
			Math.abs(targetDistance.get(NORTH)) / max.get(NORTH),
			Math.abs(targetDistance.get(EAST)) / max.get(EAST),
			Math.abs(targetDistance.get(ROTATION)) / max.get(ROTATION)
		};
		double maxScale = Math.max(scale[NORTH.ordinal()], Math.max(scale[EAST.ordinal()], scale[ROTATION.ordinal()]));
		scale = new double[]{
			scale[NORTH.ordinal()] / maxScale,
			scale[EAST.ordinal()] / maxScale,
            scale[ROTATION.ordinal()] / maxScale
		}; // NORTH - EAST
		double[] sign = {
			signum(targetDistance.get(NORTH)),
			signum(targetDistance.get(EAST)),
			signum(targetDistance.get(ROTATION))
		};

		// scale speeds and acceleration/deceleration
		fast_.put(NORTH, Math.max(min.get(NORTH), max.get(NORTH) * scale[NORTH.ordinal()]) * sign[NORTH.ordinal()]);
		fast_.put(EAST, Math.max(min.get(EAST), max.get(EAST) * scale[EAST.ordinal()]) * sign[EAST.ordinal()]);
		fast_.put(ROTATION, Math.max(min.get(ROTATION), max.get(ROTATION) * scale[ROTATION.ordinal()]) * sign[ROTATION.ordinal()]);
		
		slow_.put(NORTH, min.get(NORTH) * sign[NORTH.ordinal()]);
		slow_.put(EAST, min.get(EAST) * sign[EAST.ordinal()]);
		slow_.put(ROTATION, min.get(ROTATION) * sign[ROTATION.ordinal()]);
		// don't scale below minimum
		acceleration_ = new AccelerationLimits(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT,
											   robot_.chassis_,
											   scale);

		origin_ = robot_.chassis_.getPosition().copy();
		distanceRemaining_.put(NORTH, targetDistance.get(NORTH));
		distanceRemaining_.put(EAST, targetDistance.get(EAST));
		distanceRemaining_.put(ROTATION, targetDistance.get(ROTATION));

		finished_.put(NORTH, abs(distanceRemaining_.get(NORTH)) == 0d);
		finished_.put(EAST, abs(distanceRemaining_.get(EAST)) == 0d);
		finished_.put(ROTATION, abs(distanceRemaining_.get(ROTATION)) == 0d);

		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE DISTANCE:" +
						   targetDistance.get(NORTH) +
						   " " +
						   targetDistance.get(EAST) +
						   " " +
						   targetDistance.get(ROTATION) +
						   " (" +
						   method_ +
						   ")");
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE SCALE:" +
						   scale[NORTH.ordinal()] +
						   " " +
						   scale[EAST.ordinal()] +
						   " " +
						   scale[ROTATION.ordinal()]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE FAST:" +
						   fast_.get(NORTH) +
						   " " +
						   fast_.get(EAST) +
						   " " +
						   fast_.get(ROTATION));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE SLOW:" +
						   slow_.get(NORTH) +
						   " " +
						   slow_.get(EAST) +
						   " " +
						   slow_.get(ROTATION));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL N:" +
						   acceleration_.get(NORTH, ACCELERATION) +
						   " " +
						   acceleration_.get(NORTH, DECELERATION));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL E:" +
						   acceleration_.get(EAST, ACCELERATION) +
						   " " +
						   acceleration_.get(EAST, DECELERATION));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL R:" +
                           acceleration_.get(ROTATION, ACCELERATION) +
                           " " +
                           acceleration_.get(ROTATION, DECELERATION));
	}

	@Override
	public void execute() {
		vCurrent = robot_.chassis_.getVelocityCalculated();
		double[] vNew = {0, 0, 0};
		EnumMap<VelocityDirection, Double> distanceTraveled = (new Position(robot_.chassis_.getPosition())).distanceRelative(origin_);
		boolean[] slow = {false, false, false};
		for(VelocityDirection direction : VelocityDirection.values()){
			distanceRemaining_.put(direction, targetDistance.get(direction) - distanceTraveled.get(direction));
			double distanceToStop = vCurrent.get(direction) * vCurrent.get(direction) /
									acceleration_.get(direction, DECELERATION) /
									2.;// exact absolute value per physics
			distanceToStop += Math.max(
				distanceToStop * .05,
				Math.abs(vCurrent.get(direction)) * .04
			); // pad just a bit to avoid overshoot
			slow[direction.ordinal()] = finished_.get(direction) ||
					  Math.abs(distanceRemaining_.get(direction)) <= distanceToStop; // slow down within padded stopping distance
		}
		boolean s = Math.abs(targetDistance.get(ROTATION)) > 0 ? slow[ROTATION.ordinal()] : (slow[NORTH.ordinal()] && slow[EAST.ordinal()]);
		for(VelocityDirection direction : VelocityDirection.values()) {
			vNew[direction.ordinal()] = finished_.get(direction) ? 0d : (s ? slow_ : fast_).get(direction);
		}
/*
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2:"
    + finished_[0] + "/" + finished_[1] + "/" + finished_[2] + " " + slow[0] + "/" + slow[1] + "/" + slow[2]
    + Math.round(distanceTraveled[0]*10)/10. + "/" + Math.round(distanceTotal_[0]*10)/10. + "@" + Math.round(vNew[0]*10)/10. + "   "
    + Math.round(distanceTraveled[1]*10)/10. + "/" + Math.round(distanceTotal_[1]*10)/10. + "@" + Math.round(vNew[1]*10)/10. + "   "
    + Math.round(distanceTraveled[2]*10)/10. + "/" + Math.round(distanceTotal_[2]*10)/10. + "@" + Math.round(vNew[2]*10)/10. + robot_.angleSensor_.getAngle());
*/
		switch(method_) {
			case MOVE_AND_ROTATE:
				robot_.chassis_.setVelocity(vNew[NORTH.ordinal()], vNew[EAST.ordinal()], vNew[ROTATION.ordinal()], acceleration_);
				break;
			case MOVE:
				robot_.chassis_.setVelocity(vNew[NORTH.ordinal()], vNew[EAST.ordinal()], acceleration_);
				break;
			case ROTATE:
				robot_.chassis_.setRotation(vNew[ROTATION.ordinal()], acceleration_);
				break;
		}
	}

	@Override
	public boolean isFinished() {
		for(VelocityDirection direction : VelocityDirection.values()) {
			double remaining = distanceRemaining_.get(direction);
			double total = distanceRemaining_.get(direction);
			finished_.compute(direction, (k, v) -> v || (signum(remaining) != signum(total)));
		}
		/*boolean reachedGoal =*/
		return abs(targetDistance.get(ROTATION)) > 0 ?
			finished_.get(ROTATION) :
			finished_.get(NORTH) && finished_.get(EAST);
//		boolean stoppedMoving = false;
//		for(double velocity : vCurrent.values()) {
//			stoppedMoving = stoppedMoving || Math.abs(velocity) <= 0.1;
//		}

//		return reachedGoal && stoppedMoving;
	}
}
