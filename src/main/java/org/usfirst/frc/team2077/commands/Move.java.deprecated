/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static org.usfirst.frc.team2077.Robot.*;

public class Move extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private double north_;
  private double east_;
  private double rotation_;
  private double finalHeading;
  private double initPosition;
  private double finalPosition;
  private final double forwardAccel = 20; //constant for acceleration in inch/sec
  private final double deccelConst = 20;

  public Move(double north, double east, double rotation) {
    addRequirements(robot_.position_, robot_.heading_);
    north_ = north;
    east_ = east;
    rotation_ = rotation;
  }

  public Move(boolean relative, double north, double east, double rotation) {
    addRequirements(robot_.position_, robot_.heading_);
  }

  public Move(boolean relative, double north, double east) {
    addRequirements(robot_.position_);
  }

  public Move(boolean relative, double rotation) {
    addRequirements(robot_.heading_);
  }

  public void initialize() {
    initPosition = robot_.chassis_.getPosition()[0];
    finalPosition = robot_.chassis_.getPosition()[0] + north_;
    finalHeading = robot_.chassis_.getPosition()[2] + rotation_;
    //distance = robot_.constants_.TESTBOT_TRACK_WIDTH;
  }

  @Override
  public void execute() {
    //Calculating decceleration velocity: vInitial = Math.sqrt(-2A * dRemains)
    //Calculating accelerating velocity: vFinal = Math.sqrt(2A * dGone)
    double vAccel;
    double vDeccel;
    if (north_ > 0) {
      vAccel = 1 + Math.sqrt(2 * forwardAccel * (robot_.chassis_.getPosition()[0] - initPosition));
      vDeccel = Math.sqrt(-(2 * - deccelConst * (finalPosition - robot_.chassis_.getPosition()[0])));
    } else {
      vAccel = - 1 - Math.sqrt(2 * forwardAccel * (initPosition - robot_.chassis_.getPosition()[0]));
      vDeccel = - Math.sqrt(-(2 * - deccelConst * (robot_.chassis_.getPosition()[0] - finalPosition)));
    }
    double velocity;
    if (Math.abs(vAccel) < Math.abs(vDeccel)) {
      velocity = vAccel;
    } else {
      if (Math.abs(vDeccel) < 5) {
        velocity = Math.copySign(5, vDeccel);
      } else {
        velocity = vDeccel;
      }
    }
    System.out.println(velocity);
    //System.out.println("Accel: " + vAccel + "        Deccel: " + vDeccel);
    if (rotation_ == 0) {
        //robot_.driveChassis_.setVelocity(Math.copySign(4, north_), 0, 0);
        robot_.chassis_.setVelocity(Math.copySign(velocity, north_), 0, 0);
    } else {
      if (robot_.chassis_.getPosition()[2] < finalHeading) {
        robot_.chassis_.setVelocity(north_ * Math.abs(2 / rotation_), 0, Math.copySign(4, rotation_));
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    //Measuring the rotation only because the angle is more important than being an inch or two off
    if (rotation_ == 0) {
      if (north_ > 0) {
        if (finalPosition <= robot_.chassis_.getPosition()[0]) {
          return true;
        }
      } else {
        if (finalPosition >= robot_.chassis_.getPosition()[0]) {
          return true;
        }
      }
    } else if (rotation_ > 0) {
      if (robot_.chassis_.getPosition()[2] >= finalHeading) {
        return true;
      }
    } else { //rotation is negative
      if (robot_.chassis_.getPosition()[2] <= finalHeading) {
        return true;
      }
    }
    return false;
  }
}
