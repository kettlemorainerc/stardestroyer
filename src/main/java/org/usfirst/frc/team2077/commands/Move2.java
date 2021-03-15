/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.usfirst.frc.team2077.math.Acceleration;
import org.usfirst.frc.team2077.math.Position;

import static org.usfirst.frc.team2077.Robot.*;

import java.util.Arrays;


public class Move2 extends CommandBase {

  private final double[] distanceTotal_; // {north, east, rotation} (signed)
  private final int method_; // 1 2 or 3 (#args to setVelocity/setRotation)

  private double[] fast_; // {north, east, rotation} (signed)
  private double[] slow_; // {north, east, rotation} (signed)
  private double[][] acceleration_; // like getAccelerationLimits, but scaled

  private double[] distanceRemaining_; // {north, east, rotation} (signed)

  private boolean[] finished_; // {north, east, rotation}


  private Position origin_;

  public static double bestRotation(double rotation) {
    double remainder = Math.abs(rotation % 180);
    int halfTurns = (int)rotation / 180;
    if (halfTurns % 2 == 1) {
      return -180 + remainder;
    }
    return remainder;
  }


  public Move2(double north, double east, double rotation) {
    this(north, east, rotation, 3, robot_.position_, robot_.heading_);
    // this(north, east, ation(rotation), 3, robot_.position_, robot_.heading_);
  }

  public Move2(double north, double east) {
    this(north, east, 0, 2, robot_.position_);
  }

  public Move2(double rotation) {
    this(0, 0, rotation, 1, robot_.heading_);
  }

  private Move2(double north, double east, double rotation, int method, Subsystem... requirements) {

    addRequirements(requirements);
    // distanceTotal_ = new double[] {north, east * .68, rotation * 7/8}; //fudged values for the multipliers
    distanceTotal_ = new double[] {north, east, rotation}; //fudged values for the multipliers
    method_ = method;
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 DISTANCE:" + distanceTotal_[0] + " " + distanceTotal_[1] + " " + distanceTotal_[2] + " (" + method_ + ")");
  }


  @Override
  public void initialize() {

    double[] max = robot_.chassis_.getMaximumVelocity(); // {north, east, rotation}
    double[] min = robot_.chassis_.getMinimumVelocity(); // {north, east, rotation}

    // scale factors for north/east/rotation by fraction of maximum velocity
    double[] scale = {Math.abs(distanceTotal_[0])/max[0], Math.abs(distanceTotal_[1])/max[1], Math.abs(distanceTotal_[2])/max[2]};
    double maxScale = Math.max(scale[0], Math.max(scale[1], scale[2]));
    scale = new double[] {scale[0]/maxScale, scale[1]/maxScale, scale[2]/maxScale}; // 0 - 1
    double[] sign = {Math.signum(distanceTotal_[0]), Math.signum(distanceTotal_[1]), Math.signum(distanceTotal_[02])};

    // scale speeds and acceleration/deceleration
    fast_ = new double[] {
      Math.max(min[0], max[0]*scale[0]) * sign[0],
      Math.max(min[1], max[1]*scale[1]) * sign[1],
      Math.max(min[2], max[2]*scale[2]) * sign[2]}; // don't let maximum scale below minimum
    slow_ = new double[] {min[0] * sign[0], min[1] * sign[1], min[2] * sign[2]}; // don't scale below minimum
    acceleration_ = (new Acceleration(robot_.constants_.STARDESTROYER_ACCELERATION_G_LIMIT, robot_.constants_.STARDESTROYER_DECELERATION_G_LIMIT, robot_.chassis_, scale).get()); 

    origin_ = new Position(robot_.chassis_.getPosition());
    distanceRemaining_ = new double[] {distanceTotal_[0], distanceTotal_[1], distanceTotal_[2]};
    finished_ = new boolean[] {Math.abs(distanceRemaining_[0])==0., Math.abs(distanceRemaining_[1])==0., Math.abs(distanceRemaining_[2])==0.};

    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 DISTANCE:" + distanceTotal_[0] + " " + distanceTotal_[1] + " " + distanceTotal_[2] + " (" + method_ + ")");
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 SCALE:" + scale[0] + " " + scale[1] + " " + scale[2]);
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 FAST:" + fast_[0] + " " + fast_[1] + " " + fast_[2]);
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 SLOW:" + slow_[0] + " " + slow_[1] + " " + slow_[2]);
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 ACCEL N:" + acceleration_[0][0] + " " + acceleration_[0][1]);
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 ACCEL E:" + acceleration_[1][0] + " " + acceleration_[1][1]);
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2 ACCEL R:" + acceleration_[2][0] + " " + acceleration_[2][1]);
  }

  private double[] vCurrent;
  
  @Override
  public void execute() {

    vCurrent = robot_.chassis_.getVelocityCalculated();
    double[] vNew = {0, 0, 0};
    double[] distanceTraveled = (new Position(robot_.chassis_.getPosition())).distanceRelative(origin_);
    boolean[] slow = {false, false, false};
    for (int i = 0; i < 3; i++) {
      distanceRemaining_[i] = distanceTotal_[i] - distanceTraveled[i];
      double distanceToStop = vCurrent[i]*vCurrent[i] / acceleration_[i][1] / 2.;// exact absolute value per physics
      distanceToStop += Math.max(distanceToStop*.05, Math.abs(vCurrent[i])*.04); // pad just a bit to avoid overshoot
      slow[i] = finished_[i] || Math.abs(distanceRemaining_[i])<=distanceToStop; // slow down within padded stopping distance
    }
    boolean s = Math.abs(distanceTotal_[2])>0 ? slow[2] : (slow[0] && slow[1]);
    for (int i = 0; i < 3; i++) {
      vNew[i] = finished_[i] ? 0. : s ? slow_[i] : fast_[i];
    }
/*
    System.out.println("$$$$$$$$$$$$$$$$$$ MOVE2:"
    + finished_[0] + "/" + finished_[1] + "/" + finished_[2] + " " + slow[0] + "/" + slow[1] + "/" + slow[2]
    + Math.round(distanceTraveled[0]*10)/10. + "/" + Math.round(distanceTotal_[0]*10)/10. + "@" + Math.round(vNew[0]*10)/10. + "   "
    + Math.round(distanceTraveled[1]*10)/10. + "/" + Math.round(distanceTotal_[1]*10)/10. + "@" + Math.round(vNew[1]*10)/10. + "   "
    + Math.round(distanceTraveled[2]*10)/10. + "/" + Math.round(distanceTotal_[2]*10)/10. + "@" + Math.round(vNew[2]*10)/10. + robot_.angleSensor_.getAngle());
*/
    switch (method_) {
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
  public boolean isFinished() {
    for (int i = 0; i < 3; i++) {
      finished_[i] = finished_[i] || (Math.signum(distanceRemaining_[i]) != Math.signum(distanceTotal_[i]));
    }
    boolean reachedGoal = Math.abs(distanceTotal_[2])>0 ? finished_[2] : (finished_[0] && finished_[1]);

    boolean hasStopped = true;
    for(double velocity : vCurrent) {
      hasStopped = hasStopped && velocity <= 0.05;
    }

    return hasStopped = reachedGoal;
  }
  boolean reachedGoal = Math.abs(distanceTotal_[2])>0 ? finished_[2] : (finished_[0] && finished_[1]);
  boolean stoppedMoving = true;
  for(double velocity : vCurrent) {
    stoppedMoving = stoppedMoving && Math.abs(velocity) <= 0.1;
  }

  return reachedGoal && stoppedMoving;
}
}