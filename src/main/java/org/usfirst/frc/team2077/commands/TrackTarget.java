/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction;
import org.usfirst.frc.team2077.math.Position;

import static org.usfirst.frc.team2077.Robot.robot_;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction.*;

/**
 * Continually adjust crosshairs based on robot movement.
 */
public class TrackTarget extends CommandBase {

	private Position oldPosition_;

	boolean debug_ = true;

	public TrackTarget() {
		addRequirements(robot_.target_);
	}

	@Override
	public void initialize() {
		oldPosition_ = robot_.chassis_.getPosition();
	}

	@Override
	public void execute() {
		double[] t = robot_.crosshairs_.get();
		double n = oldPosition_.get(NORTH) + t[1] * Math.cos(Math.toRadians(t[0] + oldPosition_.get(CLOCKWISE))); // target absolute N
		double e = oldPosition_.get(EAST) + t[1] * Math.sin(Math.toRadians(t[0] + oldPosition_.get(CLOCKWISE))); // target absolute E
		Position newPosition = robot_.chassis_.getPosition();
		n -= newPosition.get(NORTH);
		e -= newPosition.get(EAST);
		double r = Math.sqrt(n * n + e * e);
		double a = Math.toDegrees(Math.atan2(e, n)) - newPosition.get(CLOCKWISE);
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
