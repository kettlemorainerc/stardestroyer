/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.usfirst.frc.team2077.Robot.*;
import org.usfirst.frc.team2077.commands.Move2;


public class MoveArc extends SequentialCommandGroup {

    private double distance_;
    private int rotation_;
    private boolean direction_;
    private double angleIncrememnt = 1;
    private double distanceIncrement;

    private MoveArc(double radius, int rotation, Subsystem requirement) {
        addRequirements(requirement);
        distance_ = radius;
        direction_ = Math.signum(rotation) == 1;
        rotation_ = (int) (rotation * Math.signum(rotation));
        distanceIncrement = distance_ / (rotation_);
    }

    private double[] getDistance(double distance, double angle) {
        double north = distance * Math.sin(angle/180*Math.PI);
        double east = distance * Math.cos(angle/180*Math.PI);
        return new double[] {north, east};
    }

    @Override
    public void initialize() {
        for (int i=0; i < rotation_; i++) {
            double[] movements = getDistance(distanceIncrement, angleIncrememnt);
            addCommands(new Move2(movements[0], movements[1]));
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
