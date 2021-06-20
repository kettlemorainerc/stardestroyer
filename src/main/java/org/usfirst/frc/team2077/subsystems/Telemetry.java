/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.subsystems;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usfirst.frc.team2077.Robot.*;

public class Telemetry extends SubsystemBase {

    Map<String,NetworkTableEntry> nte_ = new TreeMap<>();

    private NetworkTableEntry getNTE(String key) {
        if (!nte_.containsKey(key)) {
            nte_.put(key, robot_.networkTableInstance_.getEntry(key));
        }
        return nte_.get(key);
    }

    @Override
    public void periodic() {

        getNTE("Position").forceSetDoubleArray(
            robot_.chassis_.getPosition()
                           .values()
                           .stream()
                           .mapToDouble(a -> a == null ? 0 : a)
                           .toArray()
        );
        
        getNTE("Target").setDoubleArray(robot_.crosshairs_.get());
        getNTE("Crosshairs").setDoubleArray(robot_.crosshairs_.getCamera());

        //TODO: Finish below
//        if (robot_.potentialSensor_ != null) {
//            getNTE("screwyPotentiometerVoltage").setDouble(robot_.potentialSensor_.getScrewVoltage());
//        }

        if (robot_.launcher_ instanceof Launcher) {
            getNTE("RangeAV").setDoubleArray(((Launcher)robot_.launcher_).getRangeAV());
//            System.out.println(((Launcher)robot_.launcher_).getRangeAV()[0]);
            getNTE("setPointShoot").setDouble(robot_.testLauncher_.launcherRPM_);

            getNTE("ReadyShoot").setBoolean(robot_.testLauncher_.isReady());
        }
    }
}
