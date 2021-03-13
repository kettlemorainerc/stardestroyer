package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class GalacticSearch extends SequentialCommandGroup {

	public GalacticSearch(boolean a, boolean red) {
		//TODO: Check angles
		if (a) {
			if (red) {
				addCommands( //Path A red
						getMoveToPickUp(60, 0, 0),
						new Move2(0, 30),
						getMoveToPickUp(60, 0, 0),
						new Move2(0, -90),
						getMoveToPickUp(60, 0, 0),
						new Move2(180, 0)
						// getMoveToPickUp(90, 0, 0),
						// getMoveToPickUp(60, 30, 30),
						// getMoveToPickUp(30, -90, -101.56), //tan^-1((pi/2)/(pi/6)) + 30 == tan^-1(90/30) + 30
						// new Move2(71.56),
						// new Move2(180, 0)
				);
			} else {
				addCommands( //Path A blue
						getMoveToPickUp(150, 0, 0),
						new Move2(0, -90),
						getMoveToPickUp(30, 0, 0),
						new Move2(0, 30),
						getMoveToPickUp(60, 0, 0),
						new Move2(60, 0)
						// getMoveToPickUp(180, 0, 0),
						// getMoveToPickUp(30, -90, -71.56),
						// getMoveToPickUp(60, 30, 101.56),
						// new Move2(-30),
						// new Move2(90, 0)
				);
			}
		} else {
			if (red) {
				addCommands( //Path B red
						getMoveToPickUp(60, 0, 0),
						new Move2(0, 60),
						getMoveToPickUp(60, 0, 0),
						new Move2(0, -60),
						getMoveToPickUp(60, 0, 0),
						new Move2(120, 0)
						// getMoveToPickUp(60, 0, 0),
						// getMoveToPickUp(60, 60, 45),
						// getMoveToPickUp(60, -60, -90),
						// new Move2(45),
						// new Move2(120, 0)
				);
			} else {
				addCommands( //Path B blue
						getMoveToPickUp(150, 0, 0),
						new Move2(0, -60),
						getMoveToPickUp(60, 0, 0),
						new Move2(0, -60),
						getMoveToPickUp(60, 0, 0),
						new Move2(30, 0)
						// getMoveToPickUp(150, 0, 0),
						// getMoveToPickUp(60, -60, -45),
						// getMoveToPickUp(60, 60, 90),
						// new Move2(-45),
						// new Move2(30, 0)
				);
			}
		}
	}

	private static SequentialCommandGroup getMoveToPickUp(double north, double east, double rotation) {
		int grabDistance = 20;
		if (rotation != 0) {
			return new SequentialCommandGroup(
					new Move2(rotation),
					new Move2(north - grabDistance, east - grabDistance),
					new RunGrabberToggle(.3), //TODO: Look at this value
					new Move2(grabDistance, grabDistance),
					new RunGrabberToggle(0)
			);

		}
		return new SequentialCommandGroup(
				new Move2(north - grabDistance, east - grabDistance),
				new RunGrabberToggle(.3), //TODO: Look at this value
				new Move(grabDistance, grabDistance),
				new RunGrabberToggle(0)
		);
	}
}