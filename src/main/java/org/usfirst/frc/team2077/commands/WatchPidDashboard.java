package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SparkNeoDriveModule;

import static org.usfirst.frc.team2077.Robot.robot_;

public class WatchPidDashboard extends CommandBase {
    private boolean originalPid = false;
    private static final String KEY = "Using Original PID";
    public WatchPidDashboard() {
        SmartDashboard.putBoolean(KEY, originalPid);
    }

    @Override
    public void execute() {
        if(originalPid != SmartDashboard.getBoolean(KEY, false)) {
            originalPid = SmartDashboard.getBoolean(KEY, false);
            for(DriveModuleIF module : robot_.chassis_.driveModule_) {
                if(module instanceof SparkNeoDriveModule) {
                    SparkNeoDriveModule neo = (SparkNeoDriveModule) module;
                    neo.usePidStyle(originalPid);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
