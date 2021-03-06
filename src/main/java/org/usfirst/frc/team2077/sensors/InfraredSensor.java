// package frc.robot;
package org.usfirst.frc.team2077.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InfraredSensor extends SubsystemBase{
///-----Mapping-----///
// private int infraredSensePortNum = 2;
private double maxLoaded_;
private double minLoaded_;
///-----Mapping-----///

///----Instances----///
AnalogInput infraredSenseObject;


////NETWORK////TABLE////

    // Nothing here yet, will need the latest code for implmanataion.

////NETWORK///TABLE////


///----Instances----///
public InfraredSensor(int id, double maxLoaded, double minLoaded) {
        infraredSenseObject = new AnalogInput(id);
        maxLoaded_ = maxLoaded;
        minLoaded_ = minLoaded;
    }

    public double getAverageVoltage() {
        return infraredSenseObject.getAverageVoltage();
    }

    public double getInfraredVoltage(){
        return infraredSenseObject.getVoltage();
        //return infraredSenseObject.getOversampleBits();
    }


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
        //System.out.println(Math.floor(infraredSenseObject.getVoltage()* 100) / 100);
        return maxLoaded_ > infraredSenseObject.getAverageVoltage() && infraredSenseObject.getAverageVoltage() > minLoaded_;
    }
}