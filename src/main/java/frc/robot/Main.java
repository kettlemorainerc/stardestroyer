/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import org.usfirst.frc.team2077.Robot;

/**
 * Modified to start a org.usfirst.frc.team2077.Robot. Don't change anything else.
 */
public final class Main {
  private Main() {
  }

  /**
   * Modified to start a org.usfirst.frc.team2077.Robot. Don't change anything else.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
    //RobotBase.startRobot(org.usfirst.frc.team2077.Pizzabot::new);
    //RobotBase.startRobot(org.usfirst.frc.team2077.PizzaDestroyer::new);
  }
}
