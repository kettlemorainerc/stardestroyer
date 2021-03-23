/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.robot_;

/**
 * Continually adjust crosshairs based on robot movement.
 */
public class TrackTarget extends CommandBase {

	private double[] oldPosition_;

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
		double n = oldPosition_[0] + t[1] * Math.cos(Math.toRadians(t[0] + oldPosition_[2])); // target absolute N
		double e = oldPosition_[1] + t[1] * Math.sin(Math.toRadians(t[0] + oldPosition_[2])); // target absolute E
		double[] newPosition = robot_.chassis_.getPosition();
		n -= newPosition[0];
		e -= newPosition[1];
		double r = Math.sqrt(n * n + e * e);
		double a = Math.toDegrees(Math.atan2(e, n)) - newPosition[2];
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
