package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class GalacticSearch extends SequentialCommandGroup {

    private static SequentialCommandGroup getMoveToPickUp(double north, double east, double rotation) {
        int grabDistance = 20;
        // if (rotation != 0) {
        //     return new SequentialCommandGroup(
        //         new Move(rotation),
        //         new Move(north - grabDistance, 0),
        //         new RunGrabberToggle(.3), //TODO: Look at this value
        //         new Move(grabDistance, 0),
        //         new RunGrabberToggle(0)
        //     );

        // }
        return new SequentialCommandGroup(
            new Move(north - grabDistance, 0),
            new RunGrabberToggle(.6), //TODO: Look at this value
            new Move(grabDistance, 0),
            new RunGrabberToggle(0)
        );
    }
  
    public GalacticSearch(boolean a, boolean red) {
    //TODO: Check angles
    if (a) {
        if (red) {
            addCommands( //Path A red
                new RunGrabberToggle(.6),
                new Move(60, 0),
                new Move(0, 24),
                new Move(30, 0),
                new Move(-5, -70),
                new Move(172, 0),
                new RunGrabberToggle(0)
                // getMoveToPickUp(90, 0, 0),
                // getMoveToPickUp(60, 30, 30),
                // getMoveToPickUp(30, -90, -101.56), //tan^-1((pi/2)/(pi/6)) + 30 == tan^-1(90/30) + 30
                // new Move(71.56),
                // new Move(180, 0)
                );
        } else {
            addCommands( //Path A blue
                getMoveToPickUp(150, 0, 0),
                new Move(0, -90),
                getMoveToPickUp(30, 0, 0),
                new Move(0, 30),
                getMoveToPickUp(60, 0, 0),
                new Move(30, 0)
                // getMoveToPickUp(180, 0, 0),
                // getMoveToPickUp(30, -90, -71.56),
                // getMoveToPickUp(60, 30, 101.56),
                // new Move(-30),
                // new Move(90, 0)
            );
        }
    } else {
        if (red) {
            addCommands( //Path B red
                new RunGrabberToggle(.5),
                new Move(1, 0),
                new Move(0, -26.5),
                new Move(22, 0),
                new Move(0, 65),
                new Move(51, 0),
                new Move(0, -70),
                new Move(120, 0),
                new RunGrabberToggle(0)
                // getMoveToPickUp(60, 0, 0),
                // getMoveToPickUp(60, 60, 45),
                // getMoveToPickUp(60, -60, -90),
                // new Move(45),
                // new Move(120, 0)
            );
        } else {
            addCommands( //Path B blue
                getMoveToPickUp(150, 0, 0),
                new Move(0, -60),
                getMoveToPickUp(60, 0, 0),
                new Move(0, -60),
                getMoveToPickUp(60, 0, 0),
                new Move(30, 0)
                // getMoveToPickUp(150, 0, 0),
                // getMoveToPickUp(60, -60, -45),
                // getMoveToPickUp(60, 60, 90),
                // new Move(-45),
                // new Move(30, 0)
            );
        }
    }
    }
}