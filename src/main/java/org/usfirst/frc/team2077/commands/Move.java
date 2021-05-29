/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.drivetrain.MecanumMath;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction;
import org.usfirst.frc.team2077.math.Acceleration;
import org.usfirst.frc.team2077.math.Position;

import java.util.EnumMap;

import static java.lang.Math.*;
import static org.usfirst.frc.team2077.Robot.robot_;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction.*;


public class Move extends CommandBase {
	 public final double ACCELERATION_G_LIMIT = .1;
	 public final double DECELERATION_G_LIMIT = .3;

	public enum Style {
		MOVE_AND_ROTATE,
		MOVE,
		ROTATE
	}
	
	public static <T> EnumMap<Direction, T> directionMap() {
		return new EnumMap<>(Direction.class);
	}

	private final EnumMap<Direction, Double> distanceTotal_;
	private final Style method_;
	private EnumMap<Direction, Double> vCurrent = directionMap();
	private EnumMap<Direction, Double> fast_ = directionMap();
	private EnumMap<Direction, Double> slow_ = directionMap();
	private EnumMap<Direction, Double> distanceRemaining_ = directionMap();
	private EnumMap<Direction, Boolean> finished_ = directionMap();
	private double[][] acceleration_;
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
		distanceTotal_ = new EnumMap<>(Direction.class);
		distanceTotal_.put(NORTH, north);
		distanceTotal_.put(EAST, east);
		distanceTotal_.put(CLOCKWISE, rotation);
		method_ = method;
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 DISTANCE:" +
						   distanceTotal_.get(NORTH) +
						   " " +
						   distanceTotal_.get(EAST) +
						   " " +
						   distanceTotal_.get(CLOCKWISE) +
						   " (" +
						   method_ +
						   ")");
	}


	@Override
	public void initialize() {
		EnumMap<Direction, Double> max = robot_.chassis_.getMaximumVelocity();
		EnumMap<Direction, Double> min = robot_.chassis_.getMinimumVelocity();

		// scale factors for north/east/rotation by fraction of maximum velocity
		double[] scale = {
			Math.abs(distanceTotal_.get(NORTH)) / max.get(NORTH),
			Math.abs(distanceTotal_.get(EAST)) / max.get(EAST),
			Math.abs(distanceTotal_.get(CLOCKWISE)) / max.get(CLOCKWISE)
		};
		double maxScale = Math.max(scale[NORTH.ordinal()], Math.max(scale[EAST.ordinal()], scale[CLOCKWISE.ordinal()]));
		scale = new double[]{
			scale[NORTH.ordinal()] / maxScale,
			scale[EAST.ordinal()] / maxScale,
			scale[CLOCKWISE.ordinal()] / maxScale
		}; // NORTH - EAST
		double[] sign = {
			signum(distanceTotal_.get(NORTH)),
			signum(distanceTotal_.get(EAST)),
			signum(distanceTotal_.get(CLOCKWISE))
		};

		// scale speeds and acceleration/deceleration
		fast_.put(NORTH, Math.max(min.get(NORTH), max.get(NORTH) * scale[NORTH.ordinal()]) * sign[NORTH.ordinal()]);
		fast_.put(EAST, Math.max(min.get(EAST), max.get(EAST) * scale[EAST.ordinal()]) * sign[EAST.ordinal()]);
		fast_.put(CLOCKWISE, Math.max(min.get(CLOCKWISE), max.get(CLOCKWISE) * scale[CLOCKWISE.ordinal()]) * sign[CLOCKWISE.ordinal()]);
		
		slow_.put(NORTH, min.get(NORTH) * sign[NORTH.ordinal()]);
		slow_.put(EAST, min.get(EAST) * sign[EAST.ordinal()]);
		slow_.put(CLOCKWISE, min.get(CLOCKWISE) * sign[CLOCKWISE.ordinal()]);
		// don't scale below minimum
		acceleration_ = (new Acceleration(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT,
		                                  robot_.chassis_,
		                                  scale
		).get());

		origin_ = new Position(robot_.chassis_.getPosition());
		distanceRemaining_ = new EnumMap<Direction, Double>(Direction.class);
		distanceRemaining_.put(NORTH, distanceTotal_.get(NORTH));
		distanceRemaining_.put(EAST, distanceTotal_.get(EAST));
		distanceRemaining_.put(CLOCKWISE, distanceTotal_.get(CLOCKWISE));

		finished_.put(NORTH, abs(distanceRemaining_.get(NORTH)) == 0d);
		finished_.put(EAST, abs(distanceRemaining_.get(EAST)) == 0d);
		finished_.put(CLOCKWISE, abs(distanceRemaining_.get(CLOCKWISE)) == 0d);

		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE DISTANCE:" +
						   distanceTotal_.get(NORTH) +
						   " " +
						   distanceTotal_.get(EAST) +
						   " " +
						   distanceTotal_.get(CLOCKWISE) +
						   " (" +
						   method_ +
						   ")");
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE SCALE:" +
						   scale[NORTH.ordinal()] +
						   " " +
						   scale[EAST.ordinal()] +
						   " " +
						   scale[CLOCKWISE.ordinal()]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE FAST:" +
						   fast_.get(NORTH) +
						   " " +
						   fast_.get(EAST) +
						   " " +
						   fast_.get(CLOCKWISE));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE SLOW:" +
						   slow_.get(NORTH) +
						   " " +
						   slow_.get(EAST) +
						   " " +
						   slow_.get(CLOCKWISE));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL N:" +
						   acceleration_[NORTH.ordinal()][NORTH.ordinal()] +
						   " " +
						   acceleration_[NORTH.ordinal()][EAST.ordinal()]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL E:" +
						   acceleration_[EAST.ordinal()][NORTH.ordinal()] +
						   " " +
						   acceleration_[EAST.ordinal()][EAST.ordinal()]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL R:" +
						   acceleration_[CLOCKWISE.ordinal()][NORTH.ordinal()] +
						   " " +
						   acceleration_[CLOCKWISE.ordinal()][EAST.ordinal()]);
	}

	@Override
	public void execute() {
		vCurrent = robot_.chassis_.getVelocityCalculated();
		double[] vNew = {0, 0, 0};
		EnumMap<Direction, Double> distanceTraveled = (new Position(robot_.chassis_.getPosition())).distanceRelative(origin_);
		boolean[] slow = {false, false, false};
		for(Direction direction : Direction.values()){

			int i = direction.ordinal();
			distanceRemaining_.compute(direction, (k,v) -> v - distanceTraveled.get(direction));
			double distanceToStop = vCurrent.get(direction) * vCurrent.get(direction) /
									acceleration_[i][1] /
									2.;// exact absolute value per physics
			distanceToStop += Math.max(
				distanceToStop * .05,
				Math.abs(vCurrent.get(direction)) * .04
			); // pad just a bit to avoid overshoot
			slow[i] = finished_.get(direction) ||
					  Math.abs(distanceRemaining_.get(direction)) <= distanceToStop; // slow down within padded stopping distance
		}
		boolean s = Math.abs(distanceTotal_.get(CLOCKWISE)) > 0 ? slow[CLOCKWISE.ordinal()] : (slow[NORTH.ordinal()] && slow[EAST.ordinal()]);
		for(Direction direction : Direction.values()) {
			int i = direction.ordinal();
			vNew[i] = finished_.get(direction) ? 0d : (s ? slow_ : fast_).get(direction);
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
				robot_.chassis_.setVelocity(vNew[NORTH.ordinal()], vNew[EAST.ordinal()], vNew[CLOCKWISE.ordinal()], acceleration_);
				break;
			case MOVE:
				robot_.chassis_.setVelocity(vNew[NORTH.ordinal()], vNew[EAST.ordinal()], acceleration_);
				break;
			case ROTATE:
				robot_.chassis_.setRotation(vNew[CLOCKWISE.ordinal()], acceleration_);
				break;
		}
	}

	@Override
	public boolean isFinished() {
		for(Direction direction : Direction.values()) {
			int i = direction.ordinal();
			double remaining = distanceRemaining_.get(direction);
			double total = distanceRemaining_.get(direction);
			finished_.compute(direction, (k, v) -> v || (signum(remaining) != signum(total)));
		}
		boolean reachedGoal = abs(distanceTotal_.get(CLOCKWISE)) > 0 ?
			finished_.get(CLOCKWISE) :
			finished_.get(NORTH) && finished_.get(EAST);
		boolean stoppedMoving = false;
		for(double velocity : vCurrent.values()) {
			stoppedMoving = stoppedMoving || Math.abs(velocity) <= 0.1;
		}

		return reachedGoal && stoppedMoving;
	}
}
