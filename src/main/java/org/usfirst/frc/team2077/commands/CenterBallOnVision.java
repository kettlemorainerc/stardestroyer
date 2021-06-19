/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import java.util.Map;
import java.util.TreeMap;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import static org.usfirst.frc.team2077.Robot.*;


// import java.util.TreeMap;

/**
 * 
*/

/*
 ---NETWORK TABLES VALUES---





*/
public class CenterBallOnVision extends CommandBase {

  private Map<String,NetworkTableEntry> nte_ = new TreeMap<>();
  private boolean done = false;

  public CenterBallOnVision() {

  }


  private NetworkTableEntry getNTE(String key ) {
    NetworkTableEntry nte;
    if ( robot_.networkTableInstance_ != null
      && ( (nte = nte_.get(key)) != null
        || ( (nte = robot_.networkTableInstance_.getEntry(key)) != null
          && nte_.put(key, nte) == null ) ) ) {
        return nte;
    }
    return null;
}




  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    NetworkTableEntry nte;

    if ( (nte = getNTE("ball1")) != null) {
      double[] ball1 = nte.getDoubleArray(new double[0]);
      if(ball1[3] != 0.0)
        robot_.chassis_.setRotation(ball1[3]*1.25);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
