package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.robot_;

public class resetCrosshairs extends CommandBase {

    public resetCrosshairs() {
        this(robot_.constants_.UPPER_TARGET_HEIGHT);
    }

    public resetCrosshairs(double targetHeight) {
        addRequirements(robot_.launcher_);
    }

    @Override
    public void initialize() {
        robot_.crosshairs_.set(0, 0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

