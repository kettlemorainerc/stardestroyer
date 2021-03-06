// package frc.robot;
package org.usfirst.frc.team2077.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MicroSwitch extends SubsystemBase{
///-----Mapping-----///
private int infraredSensePortNum = 2;
///-----Mapping-----///

///----Instances----///
DigitalInput SwitchObject;


////NETWORK////TABLE////

    // Nothing here yet, will need the latest code for implmanataion.

////NETWORK///TABLE////


///----Instances----///
public MicroSwitch(int id) {
        SwitchObject = new DigitalInput(id);
    }


    // public double getVoltage(){
    //     return SwitchObject.get();
    // }


    public boolean getStatusIsLoaded(){
        // START TABLE
        // final double hasABallVoltage_ = 0;
        // final double hasNoBallVoltage_ = 0;
        // END TABLE
        
        // boolean tf_ = false;
        // if(maxLoaded_ > infraredSenseObject.getVoltage() && infraredSenseObject.getVoltage() > minLoaded_) {
        //     tf_ = true;
        //     System.out.println("BallDetected");
        // }

        // 0.86
        // 2.3
        //System.out.println(SwitchObject.get());
        return !SwitchObject.get();
    }
}