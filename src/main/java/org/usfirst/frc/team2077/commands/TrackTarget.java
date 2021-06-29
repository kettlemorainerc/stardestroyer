/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.math.Position;
import org.usfirst.frc.team2077.subsystems.Crosshairs;

import static org.usfirst.frc.team2077.Robot.robot_;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;

/**
 * Continually adjust crosshairs based on robot movement.
 */
public class TrackTarget extends CommandBase {

	private Position oldPosition_;
	private final Crosshairs crosshairs;
	private final AbstractChassis chassis;
	private final Subsystem target;

	boolean debug_ = true;

	public TrackTarget(AbstractChassis chassis, Crosshairs crosshairs, Subsystem target) {
		addRequirements(target);
		this.target = target;
		this.crosshairs = crosshairs;
		this.chassis = chassis;
	}

	@Override
	public void initialize() {
		oldPosition_ = robot_.chassis_.getPosition();
	}

	@Override
	public void execute() {
		double[] t = crosshairs.get();
		double n = oldPosition_.get(NORTH) + t[1] * Math.cos(Math.toRadians(t[0] + oldPosition_.get(ROTATION))); // target absolute N
		double e = oldPosition_.get(EAST) + t[1] * Math.sin(Math.toRadians(t[0] + oldPosition_.get(ROTATION))); // target absolute E
		Position newPosition = chassis.getPosition();
		n -= newPosition.get(NORTH);
		e -= newPosition.get(EAST);
		double r = Math.sqrt(n * n + e * e);
		double a = Math.toDegrees(Math.atan2(e, n)) - newPosition.get(ROTATION);
		robot_.crosshairs_.set(a, r);
		oldPosition_ = newPosition;
	}

	// speed = maxRPM * (targetAngle / (Math.min(targetAngle, Math.abs(heading)) * Math.signum(heading))

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
