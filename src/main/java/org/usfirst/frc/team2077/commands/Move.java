/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection;
import org.usfirst.frc.team2077.math.*;
import org.usfirst.frc.team2077.math.AccelerationLimits.Type;

import java.util.*;

import static org.usfirst.frc.team2077.Robot.*;


public class Move extends CommandBase {
	public static final double ACCELERATION_G_LIMIT = .1;
	public static final double DECELERATION_G_LIMIT = .3;
	private static final double STOP_WITHIN = .5;
	private final double[] distanceTotal_; // {north, east, rotation} (signed)
	private final int method_; // 1 2 or 3 (#args to setVelocity/setRotation)

	private double[] fast_; // {north, east, rotation} (signed)
	private double[] slow_; // {north, east, rotation} (signed)
	private AccelerationLimits acceleration_; // like getAccelerationLimits, but scaled

	private double[] distanceRemaining_; // {north, east, rotation} (signed)

	private boolean[] finished_; // {north, east, rotation}


	private Position origin_;

	public Move(double north, double east, double rotation) {
		this(north, east, rotation, 3, robot_.position_, robot_.heading_);
		// this(north, east, ation(rotation), 3, robot_.position_, robot_.heading_);
	}

	public Move(double north, double east) {
		this(north, east, 0, 2, robot_.position_);
	}

	public Move(double rotation) {
		this(0, 0, rotation, 1, robot_.heading_);
	}

	private Move(double north, double east, double rotation, int method, Subsystem... requirements) {

		addRequirements(requirements);
		distanceTotal_ = new double[]{north, east * .68, rotation * 7 / 8}; //fudged values for the multipliers
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

		EnumMap<VelocityDirection, Double> max = robot_.chassis_.getMaximumVelocity();
		EnumMap<VelocityDirection, Double> min = robot_.chassis_.getMinimumVelocity();

		// scale factors for north/east/rotation by fraction of maximum velocity
		double[] scale = {
			Math.abs(distanceTotal_[0]) / max.get(VelocityDirection.NORTH),
			Math.abs(distanceTotal_[1]) / max.get(VelocityDirection.EAST),
			Math.abs(distanceTotal_[2]) / max.get(VelocityDirection.ROTATION)
		};
		double maxScale = Math.max(scale[0], Math.max(scale[1], scale[2]));
		scale = new double[]{scale[0] / maxScale, scale[1] / maxScale, scale[2] / maxScale}; // 0 - 1
		double[] sign = {
			Math.signum(distanceTotal_[0]),
			Math.signum(distanceTotal_[1]),
			Math.signum(distanceTotal_[02])
		};

		// scale speeds and acceleration/deceleration
		fast_ = new double[]{
			Math.max(min.get(VelocityDirection.NORTH), max.get(VelocityDirection.NORTH) * scale[0]) * sign[0],
			Math.max(min.get(VelocityDirection.EAST), max.get(VelocityDirection.EAST) * scale[1]) * sign[1],
			Math.max(min.get(VelocityDirection.ROTATION), max.get(VelocityDirection.ROTATION) * scale[2]) * sign[2]
		}; // don't let maximum scale below minimum
		slow_ = new double[]{
			min.get(VelocityDirection.NORTH) * sign[0],
			min.get(VelocityDirection.EAST) * sign[1],
			min.get(VelocityDirection.ROTATION) * sign[2]
		}; // don't scale below minimum
		acceleration_ = new AccelerationLimits(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT, robot_.chassis_, scale);

		origin_ = new Position(robot_.chassis_.getPosition());
		distanceRemaining_ = new double[]{distanceTotal_[0], distanceTotal_[1], distanceTotal_[2]};
		finished_ = new boolean[]{
			Math.abs(distanceRemaining_[0]) == 0.,
			Math.abs(distanceRemaining_[1]) == 0.,
			Math.abs(distanceRemaining_[2]) == 0.
		};

		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 DISTANCE:" +
						   distanceTotal_[0] +
						   " " +
						   distanceTotal_[1] +
						   " " +
						   distanceTotal_[2] +
						   " (" +
						   method_ +
						   ")");
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 SCALE:" + scale[0] + " " + scale[1] + " " + scale[2]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 FAST:" + fast_[0] + " " + fast_[1] + " " + fast_[2]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 SLOW:" + slow_[0] + " " + slow_[1] + " " + slow_[2]);
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 ACCEL N:" +
						   acceleration_.get(VelocityDirection.NORTH, Type.ACCELERATION) +
						   " " +
						   acceleration_.get(VelocityDirection.NORTH, Type.DECELERATION));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 ACCEL E:" +
						   acceleration_.get(VelocityDirection.EAST, Type.ACCELERATION) +
						   " " +
						   acceleration_.get(VelocityDirection.EAST, Type.DECELERATION));
		System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 ACCEL R:" +
						   acceleration_.get(VelocityDirection.ROTATION, Type.ACCELERATION) +
						   " " +
						   acceleration_.get(VelocityDirection.ROTATION, Type.DECELERATION));
	}

	@Override
	public void execute() {

		EnumMap<VelocityDirection, Double> vCurrent = robot_.chassis_.getVelocityCalculated();
		double[] vNew = {0, 0, 0};
		EnumMap<VelocityDirection, Double> distanceTraveled = (new Position(robot_.chassis_.getPosition())).distanceRelative(
			origin_);
		boolean[] slow = {false, false, false};
		for(int i = 0; i < 3; i++) {
			VelocityDirection direction = VelocityDirection.values()[i];
			distanceRemaining_[i] = distanceTotal_[i] - distanceTraveled.get(direction);
			double distanceToStop = vCurrent.get(direction) * vCurrent.get(direction) /
									acceleration_.get(direction, Type.DECELERATION) /
									2.;// exact absolute value per physics
			distanceToStop += Math.max(
				distanceToStop * .05,
				Math.abs(vCurrent.get(direction)) * .04
			); // pad just a bit to avoid overshoot
			slow[i] = finished_[i] ||
					  Math.abs(distanceRemaining_[i]) <= distanceToStop; // slow down within padded stopping distance
		}
		boolean s = Math.abs(distanceTotal_[2]) > 0 ? slow[2] : (slow[0] && slow[1]);
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
			case 3:
				robot_.chassis_.setVelocity(vNew[0], vNew[1], vNew[2], acceleration_);
				break;
			case 2:
				robot_.chassis_.setVelocity(vNew[0], vNew[1], acceleration_);
				break;
			case 1:
				robot_.chassis_.setRotation(vNew[2], acceleration_);
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		robot_.chassis_.halt();
	}

	@Override
	public boolean isFinished() {
		for(int i = 0; i < 3; i++) {
			finished_[i] = finished_[i] ||
//			               distanceRemaining_[i] < STOP_WITHIN * Math.signum(distanceTotal_[i]) ||
			               (Math.signum(distanceRemaining_[i]) != Math.signum(distanceTotal_[i]));
		}
		return Math.abs(distanceTotal_[2]) > 0 ? finished_[2] : (finished_[0] && finished_[1]);
	}
}
