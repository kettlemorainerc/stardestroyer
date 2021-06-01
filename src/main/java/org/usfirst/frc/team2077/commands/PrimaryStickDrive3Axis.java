/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc.team2077.DriveStation;

import static org.usfirst.frc.team2077.Robot.robot_;

public class PrimaryStickDrive3Axis extends CommandBase {
	public final double ACCELERATION_G_LIMIT = .4;
	public final double DECELERATION_G_LIMIT = ACCELERATION_G_LIMIT; //1e10 //.35 is the value used for the 03-05-21 version

	public PrimaryStickDrive3Axis() {
		addRequirements(robot_.position_);
	}

	@Override
	public void execute() {

		// Speed limit as a percentage (0.0-1.0) of maximum wheel speed
		double speedLimit = 1.0;
		// Rotation limit as a percentage (0.0-1.0) of maximum wheel speed
		double rotationLimit = 1.0; // 0.3;
		// Acceleration/deceleration limit, in Gs
		// Should be at or below the static coefficient of friction (CoF) between the wheels and the floor
		// Too high allows wheelspin, too low is hazardous due to slow deceleration
		//double accelerationLimit = 1.0;
		double accelerationLimit = ACCELERATION_G_LIMIT;

		if(robot_.analogSettings_ != null) {
			double[] dialSetting = { // analog input dials, scaled to 0.0 - 1.0
				robot_.analogSettings_.get(3),
				robot_.analogSettings_.get(2),
				robot_.analogSettings_.get(1)
			};

			double speedLimitMin = 0.2;
			double speedLimitMax = 1.0;
			speedLimit = speedLimitMin + (speedLimitMax - speedLimitMin) * dialSetting[0];

			double rotationLimitMin = 0.2;
			double rotationLimitMax = 1.0;
			rotationLimit = rotationLimitMin + (rotationLimitMax - rotationLimitMin) * dialSetting[1];

			double accelerationLimitMin = .05;
			double accelerationLimitMax = 0.5;
			accelerationLimit = accelerationLimitMin +
								(accelerationLimitMax - accelerationLimitMin) * (1 - dialSetting[2]); // reverse dial
			//decelerationLimit = Math.max(accelerationLimit, .25); // don't let this go too low for safety
		}
//		double throttle = 1 - robot_.driveStation_.secondaryStick_.getRawAxis(2);
		double throttle = 1;

		robot_.chassis_.setGLimits(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT);

		// TODO: Who handles rotation updates if another command owns robot_position_?
		// TODO: Check joystick/drive capabilities and merge w/2-axis.
		double north = DriveStation.adjustInputSensitivity(robot_.driveStation_.Flight.getY(), .3, 1);
		double east = DriveStation.adjustInputSensitivity(robot_.driveStation_.Flight.getX(), .3, 1);
//		 double north = DriveStation.adjustInputSensitivity(-robot_.driveStation_.primaryStick_.getY(), .2, 2.5);
//		 double east = DriveStation.adjustInputSensitivity(robot_.driveStation_.primaryStick_.getX(), .2, 2.5);
//		north = Math.abs(north) >= Math.abs(east) ? north : 0;
//		east = Math.abs(east) > Math.abs(north) ? east : 0;

		double z = robot_.driveStation_.Flight.getZ();
		if (z > .5) (new RunGrabber(1)).schedule();
		else (new StopGrabber()).schedule();
//		else if (z < -.5) {
//
//		}


		if(CommandScheduler.getInstance().requiring(robot_.heading_) != null) { // we don't control heading
			//System.out.println(" STICK(3): " + north + " \t" + east);
			robot_.chassis_.setVelocity01(north * speedLimit * throttle, east * speedLimit * throttle);
		} else { // we control heading
//			 double clockwise = DriveStation.adjustInputSensitivity(robot_.driveStation_.primaryStick_.getRawAxis(2), .2, 2.5);
			double clockwise = DriveStation.adjustInputSensitivity(robot_.driveStation_.Flight.getRawAxis(4), .05, 1);

			if (north == 0 && east == 0 && clockwise == 0) {
//				System.out.println("Halting");
				robot_.chassis_.halt();
			} else {
//				System.out.println("Moving");
				robot_.chassis_.setVelocity01(
						north * speedLimit * throttle,
						east * speedLimit * throttle,
						clockwise * rotationLimit * throttle
				);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
