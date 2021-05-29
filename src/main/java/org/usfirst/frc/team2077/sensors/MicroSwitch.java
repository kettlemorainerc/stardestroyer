// package frc.robot;
package org.usfirst.frc.team2077.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MicroSwitch extends SubsystemBase{

    private int infraredSensePortNum = 9;
    DigitalInput SwitchObject;

    public MicroSwitch(int id) {
        SwitchObject = new DigitalInput(id);
    }


    public boolean getStatusIsHit(){
        return !SwitchObject.get();
    }
}