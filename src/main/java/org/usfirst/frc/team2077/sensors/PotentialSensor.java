// package frc.robot;
package org.usfirst.frc.team2077.sensors;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;

import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class PotentialSensor extends SubsystemBase{
///-----Mapping-----///
    int screwSencePortNum = 2;//This is the DIO port number that the potentiometer that reads the screw's posistion
    int screwMotorPortNum = 2;

    boolean reversed = false;//This is the setting if the potentiometer get reversed hardwarewise [Currently unoperational]
    boolean runWithManualPortSwitching = true;
    double currentlyAtRaw;
    //RANGES//
    static int minGiveableUnit = 0;
    static int maxGiveableUnit = 32;
    static int minScrewValueRaw = 25;
    static int maxScrewValueRaw = 4050;
    static int adjustedScrewValueRawRange = maxScrewValueRaw - minScrewValueRaw;
// /*Conversion*/ double translateOperatorGTR = (maxScrewValueRaw - minScrewValueRaw) / (maxGiveableUnit - minGiveableUnit);//Ranged multiplyer for Givable to Raw
///-----Mapping-----///

///----Instances----///
// Joystick joystick1 = new Joystick(0);
// AnalogInput screwySenseObject = new AnalogInput(screwSencePortNum);//Initializes a DigitalInput on given DIO port
AnalogInput screwySenseObject = new AnalogInput(3);//Initializes a DigitalInput on given DIO port
//Talon screwyTalonObject = new Talon(1);//screwMotorPortNum);////Initializes a DigitalInput on PWM given

// STATIC PORT OPERATIONS FOR MANUAL SWITCHING CONTROL
// Talon screwyStatic8 = new Talon(8);
// Talon screwyStatic9 = new Talon(9);


////NETWORK////TABLE////
NetworkTableInstance networkTableInstance_ = NetworkTableInstance.getDefault();
NetworkTableInstance networkTable1 = NetworkTableInstance.getDefault();//Default table created automatical (copied from documentation).
NetworkTableEntry screwyPotentiometerValue; //A value on the Data Table
////NETWORK///TABLE////


// networkTableEntry_ = robot_.networkTableInstance_.getEntry("Crosshairs");
// networkTableEntry_.setDoubleArray(new double[] {pixelX_, pixelY_, pixelWidth_, pixelHeight_});

///----Instances----///


public NetworkTableEntry networkTableEntry_;
boolean soIOnlyRunOnece = true;

    public void operation(double degreeToGetTo_){//I NEED A NEW NAME
        if(soIOnlyRunOnece && runWithManualPortSwitching){
            // screwyStatic8.set(0.99);
            // screwyStatic9.set(-0.99);
            soIOnlyRunOnece = false;
        }
        // System.out.println(screwSenseObject.getValue());


    }

    public double getScrewVoltage(){
        return screwySenseObject.getVoltage();
    }

    public double getScrewValue(){
        //System.out.println("Printed: " + screwySenseObject.getValue());
        // screwySenseObject.getValue();
        return screwySenseObject.getValue();
    }
    public int getAdjustedScrewValue(){
        // int adjustedScrewValue = screwySenseObject.getValue() - minScrewValueRaw;
        int adjustedScrewValue = screwySenseObject.getValue() - minScrewValueRaw;
        if(adjustedScrewValue >= 0){
            return(adjustedScrewValue);
        } else {
            return(0);//This should never run!
        }
    }

    public void setOnNetwork(){
        //System.out.println("Look2");
        // networkTableEntry_.setDouble("screwyPotentiometerVoltage");
        // Name launcherVoltage

        // networkTableEntry_ = robot_.networkTableInstance_.getEntry("screwyPotentiometerVoltage");
        //networkTableEntry_ = robot_.networkTableInstance_.getEntry("screwyPotentiometerVoltage");
        // networkTableEntry_.setDouble(getScrewValue());

        // screwyPotentiometerValue
        // screwyPotentiometerVoltage
        System.out.println(robot_.networkTableInstance_.getEntry("screwyPotentiometerVoltage"));
    }

    public void testing(){
        System.out.println(robot_.networkTableInstance_.getEntry("screwyPotentiometerVoltage"));
    }


    // THIS IS UNFINISHED AND SHOULD NOT BE USED
    // public double getScrewPosistion(){
    //     // This is the meothed to check agnest the tables.
    //     return 0;
    // }
}