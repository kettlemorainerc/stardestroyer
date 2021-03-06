package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class GalacticSearch extends SequentialCommandGroup {

    private static SequentialCommandGroup getMoveToPickUp(double north, double east, double rotation) {
        int grabDistance = 20;
        return new SequentialCommandGroup(
            new Move(north - grabDistance, east - grabDistance, rotation),
            new RunGrabberToggle(.3), //TODO: Look at this value
            new Move(grabDistance, grabDistance),
            new RunGrabberToggle(0)
        );
    }
  
    public GalacticSearch(boolean a, boolean red) {
    //TODO: Check angles
    if (a) {
        if (red) {
            addCommands( //Path A red
                getMoveToPickUp(90, 0, 0),
                getMoveToPickUp(60, 30, 30),
                getMoveToPickUp(30, -90, -101.56), //tan^-1((pi/2)/(pi/6)) + 30 == tan^-1(90/30) + 30
                new Move(180, 0, 71.56)
                );
        } else {
            addCommands( //Path A blue
                getMoveToPickUp(180, 0, 0),
                getMoveToPickUp(30, -90, -71.56),
                getMoveToPickUp(60, 30, 101.56),
                new Move(90, 0, -30)
            );
        }
    } else {
        if (red) {
            addCommands( //Path B red
                getMoveToPickUp(60, 0, 0),
                getMoveToPickUp(60, 60, 45),
                getMoveToPickUp(60, -60, -90),
                new Move(120, 0, 45)
            );
        } else {
            addCommands(new Move(60, 0, 0));
            // addCommands( //Path B blue
            //     getMoveToPickUp(150, 0, 0),
            //     getMoveToPickUp(60, -60, -45),
            //     getMoveToPickUp(60, 60, 90),
            //     new Move2(30, 0, -45)
            // );
        }
    }
    }
}