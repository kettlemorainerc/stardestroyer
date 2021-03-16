/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;

public class DriveStation {

    public final Joystick primaryStick_ = new Joystick(0);
    public final Joystick secondaryStick_ = new Joystick(1);
    public final Joystick testingStick_ = new Joystick(5);
    public final Joystick Flight = new Joystick(2);



    
    
    public final JoystickButton primaryTrigger_ = new JoystickButton(primaryStick_, 1); 
    public final JoystickButton primary2_ = new JoystickButton(primaryStick_, 2);
    public final JoystickButton primary3_ = new JoystickButton(primaryStick_, 3);
    public final JoystickButton primary4_ = new JoystickButton(primaryStick_, 4);
    public final JoystickButton primary5_ = new JoystickButton(primaryStick_, 5);
    public final JoystickButton primary6_ = new JoystickButton(primaryStick_, 6);
    public final JoystickButton primary7_ = new JoystickButton(primaryStick_, 7);
    public final JoystickButton primary8_ = new JoystickButton(primaryStick_, 8);
    public final JoystickButton primary9_ = new JoystickButton(primaryStick_, 9);
    public final JoystickButton primary10_ = new JoystickButton(primaryStick_, 10);
    public final JoystickButton primary11_ = new JoystickButton(primaryStick_, 11);
    public final JoystickButton primary12_ = new JoystickButton(primaryStick_, 12);

    public final JoystickButton secondaryTrigger_ = new JoystickButton(secondaryStick_, 1); 
    public final JoystickButton secondary2_ = new JoystickButton(secondaryStick_, 2);
    public final JoystickButton secondary3_ = new JoystickButton(secondaryStick_, 3);
    public final JoystickButton secondary4_ = new JoystickButton(secondaryStick_, 4);
    public final JoystickButton secondary5_ = new JoystickButton(secondaryStick_, 5);
    public final JoystickButton secondary6_ = new JoystickButton(secondaryStick_, 6);
    public final JoystickButton secondary7_ = new JoystickButton(secondaryStick_, 7);
    public final JoystickButton secondary8_ = new JoystickButton(secondaryStick_, 8);
    public final JoystickButton secondary9_ = new JoystickButton(secondaryStick_, 9);
    public final JoystickButton secondary10_ = new JoystickButton(secondaryStick_, 10);
    public final JoystickButton secondary11_ = new JoystickButton(secondaryStick_, 11);
    public final JoystickButton secondary24_ = new JoystickButton(secondaryStick_, 24);//AJ CHANGES (1.4)
   
    public final JoystickButton testing1_ = new JoystickButton(testingStick_, 1);
    public final JoystickButton testing2_ = new JoystickButton(testingStick_, 2);
    public final JoystickButton testing3_ = new JoystickButton(testingStick_, 3);
    public final JoystickButton testing4_ = new JoystickButton(testingStick_, 4);
    public final JoystickButton testing5_ = new JoystickButton(testingStick_, 5);
    public final JoystickButton testing6_ = new JoystickButton(testingStick_, 6);
    public final JoystickButton testing7_ = new JoystickButton(testingStick_, 7);
    public final JoystickButton testing8_ = new JoystickButton(testingStick_, 8);
    public final JoystickButton testing9_ = new JoystickButton(testingStick_, 9);
    public final JoystickButton testing10_ = new JoystickButton(testingStick_, 10);
    public final JoystickButton testing11_ = new JoystickButton(testingStick_, 11);
    public final JoystickButton testing12_ = new JoystickButton(testingStick_, 12);
    public final JoystickButton testing13_ = new JoystickButton(testingStick_, 13);
    public final JoystickButton testing14_ = new JoystickButton(testingStick_, 14);
    public final JoystickButton testing15_ = new JoystickButton(testingStick_, 15);
    public final JoystickButton testing16_ = new JoystickButton(testingStick_, 16);
    public final JoystickButton testing17_ = new JoystickButton(testingStick_, 17);
    public final JoystickButton testing18_ = new JoystickButton(testingStick_, 18);
    public final JoystickButton testing19_ = new JoystickButton(testingStick_, 19);
    public final JoystickButton testing20_ = new JoystickButton(testingStick_, 20);
    public final JoystickButton testing21_ = new JoystickButton(testingStick_, 21);
    public final JoystickButton testing22_ = new JoystickButton(testingStick_, 22);
    public final JoystickButton testing23_ = new JoystickButton(testingStick_, 23);
    public final JoystickButton testing24_ = new JoystickButton(testingStick_, 24);

    /**
     * Condition control axis input to improve driveability.
     * Each axis has a center dead band in which the output for that axis is always zero.
     * Outside the dead band the output increases exponentially from zero to 1 or -1.
     * @param input
     * @param deadBand
     * @param exponent
     * @return
     */
    public static double adjustInputSensitivity(double input, double deadBand, double exponent) {
        return Math.pow(Math.max(0, Math.abs(input) - deadBand) / (1 - deadBand), exponent) * Math.signum(input);
    }
}
