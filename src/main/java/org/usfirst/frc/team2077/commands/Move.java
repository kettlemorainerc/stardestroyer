/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.math.Acceleration;
import org.usfirst.frc.team2077.math.Position;

import static org.usfirst.frc.team2077.Robot.robot_;


public class Move extends CommandBase {
	 public final double ACCELERATION_G_LIMIT = .1;
	 public final double DECELERATION_G_LIMIT = .3;

	public enum Style {
		MOVE_AND_ROTATE,
		MOVE,
		ROTATE
	}

	private static final byte NORTH = 0, EAST = 1, CLOCKWISE = 2;
	private final double[] distanceTotal_;
	private final Style method_;
	private double[] vCurrent;
	private double[] fast_;
	private double[] slow_;
	private double[] distanceRemaining_;
	private boolean[] finished_;
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
		distanceTotal_ = new double[]{north, east, rotation}; //fudged values for the multipliers
		method_ = method;
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 DISTANCE:" +
						   distanceTotal_[0] +
						   " " +
						   distanceTotal_[1] +
						   " " +
						   distanceTotal_[2] +
						   " (" +
						   method_ +
						   ")");
	}


	@Override
	public void initialize() {
		double[] max = robot_.chassis_.getMaximumVelocity();
		double[] min = robot_.chassis_.getMinimumVelocity();

		// scale factors for north/east/rotation by fraction of maximum velocity
		double[] scale = {
			Math.abs(distanceTotal_[NORTH]) / max[NORTH],
			Math.abs(distanceTotal_[EAST]) / max[EAST],
			Math.abs(distanceTotal_[CLOCKWISE]) / max[CLOCKWISE]
		};
		double maxScale = Math.max(scale[NORTH], Math.max(scale[EAST], scale[CLOCKWISE]));
		scale = new double[]{
			scale[NORTH] / maxScale,
			scale[EAST] / maxScale,
			scale[CLOCKWISE] / maxScale
		}; // NORTH - EAST
		double[] sign = {
			Math.signum(distanceTotal_[NORTH]),
			Math.signum(distanceTotal_[EAST]),
			Math.signum(distanceTotal_[CLOCKWISE])
		};

		// scale speeds and acceleration/deceleration
		fast_ = new double[]{
			Math.max(min[NORTH], max[NORTH] * scale[NORTH]) * sign[NORTH],
			Math.max(min[EAST], max[EAST] * scale[EAST]) * sign[EAST],
			Math.max(min[CLOCKWISE], max[CLOCKWISE] * scale[CLOCKWISE]) * sign[CLOCKWISE]
		}; // don't let maximum scale below minimum
		slow_ = new double[]{
			min[NORTH] * sign[NORTH],
			min[EAST] * sign[EAST],
			min[CLOCKWISE] * sign[CLOCKWISE]
		}; // don't scale below minimum
		acceleration_ = (new Acceleration(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT,
		                                  robot_.chassis_,
		                                  scale
		).get());

		origin_ = new Position(robot_.chassis_.getPosition());
		distanceRemaining_ = new double[]{distanceTotal_[NORTH], distanceTotal_[EAST], distanceTotal_[CLOCKWISE]};
		finished_ = new boolean[]{
			Math.abs(distanceRemaining_[NORTH]) == 0.,
			Math.abs(distanceRemaining_[EAST]) == 0.,
			Math.abs(distanceRemaining_[CLOCKWISE]) == 0.
		};

		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE DISTANCE:" +
						   distanceTotal_[NORTH] +
						   " " +
						   distanceTotal_[EAST] +
						   " " +
						   distanceTotal_[CLOCKWISE] +
						   " (" +
						   method_ +
						   ")");
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE SCALE:" +
						   scale[NORTH] +
						   " " +
						   scale[EAST] +
						   " " +
						   scale[CLOCKWISE]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE FAST:" +
						   fast_[NORTH] +
						   " " +
						   fast_[EAST] +
						   " " +
						   fast_[CLOCKWISE]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE SLOW:" +
						   slow_[NORTH] +
						   " " +
						   slow_[EAST] +
						   " " +
						   slow_[CLOCKWISE]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL N:" +
						   acceleration_[NORTH][NORTH] +
						   " " +
						   acceleration_[NORTH][EAST]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL E:" +
						   acceleration_[EAST][NORTH] +
						   " " +
						   acceleration_[EAST][EAST]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVECLOCKWISE ACCEL R:" +
						   acceleration_[CLOCKWISE][NORTH] +
						   " " +
						   acceleration_[CLOCKWISE][EAST]);
	}

	@Override
	public void execute() {
		vCurrent = robot_.chassis_.getVelocityCalculated();
		double[] vNew = {0, 0, 0};
		double[] distanceTraveled = (new Position(robot_.chassis_.getPosition())).distanceRelative(origin_);
		boolean[] slow = {false, false, false};
		for(int i = 0; i < 3; i++) {
			distanceRemaining_[i] = distanceTotal_[i] - distanceTraveled[i];
			double distanceToStop = vCurrent[i] * vCurrent[i] /
									acceleration_[i][1] /
									2.;// exact absolute value per physics
			distanceToStop += Math.max(
				distanceToStop * .05,
				Math.abs(vCurrent[i]) * .04
			); // pad just a bit to avoid overshoot
			slow[i] = finished_[i] ||
					  Math.abs(distanceRemaining_[i]) <= distanceToStop; // slow down within padded stopping distance
		}
		boolean s = Math.abs(distanceTotal_[CLOCKWISE]) > 0 ? slow[CLOCKWISE] : (slow[NORTH] && slow[EAST]);
		for(int i = 0; i < 3; i++) {
			vNew[i] = finished_[i] ? 0. : s ? slow_[i] : fast_[i];
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
				robot_.chassis_.setVelocity(vNew[NORTH], vNew[EAST], vNew[CLOCKWISE], acceleration_);
				break;
			case MOVE:
				robot_.chassis_.setVelocity(vNew[NORTH], vNew[EAST], acceleration_);
				break;
			case ROTATE:
				robot_.chassis_.setRotation(vNew[CLOCKWISE], acceleration_);
				break;
		}
	}

	@Override
	public boolean isFinished() {
		for(int i = 0; i < 3; i++) {
			finished_[i] = finished_[i] || (Math.signum(distanceRemaining_[i]) != Math.signum(distanceTotal_[i]));
		}
		boolean reachedGoal = Math.abs(distanceTotal_[2]) > 0 ? finished_[2] : (finished_[0] && finished_[1]);
		boolean stoppedMoving = false;
		for(double velocity : vCurrent) {
			stoppedMoving = stoppedMoving || Math.abs(velocity) <= 0.1;
		}

		return reachedGoal && stoppedMoving;
	}
}
