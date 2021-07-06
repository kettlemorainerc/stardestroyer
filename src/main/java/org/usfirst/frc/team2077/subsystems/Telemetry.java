/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.subsystems;

import java.util.*;

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

    private void checkValue(String key, double[] update, double allowedDifference) {
        double[] differences = new double[update.length];
        Arrays.fill(differences, allowedDifference);
        checkValue(key, update, differences);
    }

    private void checkValue(String key, double[] update, double[] allowedDifference) {
        NetworkTableEntry entry = getNTE(key);
        double[] existing = entry.getDoubleArray(new double[0]);
        boolean equal = existing.length == update.length;
        int i = 0;
        while(equal && i < existing.length)
            equal = existing[i] - allowedDifference[i] <= update[i] &&
                    update [i] <= existing[i] + allowedDifference[i++];

        if(!Arrays.equals(update, existing)) entry.setDoubleArray(update);
    }

    @Override
    public void periodic() {
        checkValue(
            "Position",
            robot_.chassis_.getPosition()
                           .values()
                           .stream()
                           .mapToDouble(a -> a == null ? 0 : a)
                           .toArray(),
            new double[]{.1, .1, .05}
        );
        checkValue(
          "Target",
          robot_.crosshairs_.get(),
          .05
        );
        checkValue(
            "Crosshairs",
            robot_.crosshairs_.getCamera(),
            0
        );
        
//        getNTE("Target").setDoubleArray(robot_.crosshairs_.get());
//        getNTE("Crosshairs").setDoubleArray(robot_.crosshairs_.getCamera());

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
