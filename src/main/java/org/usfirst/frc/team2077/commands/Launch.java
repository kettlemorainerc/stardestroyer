package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.robot_;

public class Launch extends CommandBase {

    public Launch() {
        addRequirements(robot_.launcher_);
    }


    @Override
    public void initialize() {
        robot_.launcher_.launch();
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
