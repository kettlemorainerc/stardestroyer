package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

public class ZeroScrew extends CommandBase {

    boolean done = false;

    public ZeroScrew() {
        addRequirements(robot_.testLauncher_);
    }

    @Override
    public void initialize() {
        // robot_.testLauncher_.setScrewPosition01(up_ ? 1. : 0.0);
    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("testing?", currentPosition + (up_ ? .1 : -.1));
        robot_.testLauncher_.setScrewPosition(-4800*5000, false);
        robot_.testLauncher_.zeroScrew();
        done = true;
    }

    @Override
    public void end(boolean interrupted) {
        // robot_.testLauncher_.setScrewPosition(robot_.testLauncher_.getScrewPosition()); // stop when button is released
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
