package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.*;

public class
AutoNavRoutes extends SequentialCommandGroup {
	///---Mapping---///
	final private static double NORTH_MULTIPLIER = 1.0;
	final private static double EAST_MULTIPLIER = 1.0;
	final private static double ROTATION_MULTIPLIER = 1.0;
///---Mapping---///

//TODO: Calabrate

	public AutoNavRoutes(AutoNavRoute path, Subsystem position, Subsystem heading) {
		switch(path) {
			case BARREL_RACE:
				barrelRace(position, heading).schedule();
				break;
			case SLALOM:
				slalomPath(position).schedule();
				break;
			case BOUNCE:
				bouncePath(position).schedule();
				break;
			default:
				System.out.println("\n   [WARNING]: Else reached for AutoNavRoutes - '26893223421231'\n");//Random string of numbers, so it can be searched in the code
		}
	}

	//Calibration values needed.
	private static double m3N(double north_) {//move3N
		return north_ * NORTH_MULTIPLIER;
	}

	private static double m3E(double east_) {
		return east_ * EAST_MULTIPLIER;
	}

	private static double m3R(double rotation_) {
		return rotation_ * ROTATION_MULTIPLIER;
	}

	private static SequentialCommandGroup barrelRaceOrig(Subsystem position, Subsystem heading) {
		double _unit = 21.5;
		return (new SequentialCommandGroup(
			new Move(m3N(16.5 + 90.0), m3E(0.0), position),
			new Move(m3N(_unit), m3E(_unit), position),
			new Move(m3N(-_unit), m3E(_unit), position),
			new Move(m3N(-_unit), m3E(_unit), position),
			new Move(m3N(_unit), m3E(-_unit), position),
			new Move(m3R(-29.05), heading),
			new Move(m3N(102.956), m3E(0.0), position),
			new Move(m3N(0.0), m3E(-_unit), position),
			new Move(m3N(-_unit), m3E(0.0), position),
			new Move(m3N(94.22), m3E(_unit), position),
			new Move(m3N(_unit), m3E(_unit), position),
			new Move(m3N(_unit), m3E(-_unit), position),
			new Move(m3N(0.0), m3E(_unit), position),

			new Move(m3N(300.0), m3E(300.0), position)
		)
		);
	}

	private static SequentialCommandGroup barrelRace(Subsystem position, Subsystem heading) {
		return (new SequentialCommandGroup(
			new Move(m3N(135.0), m3E(0.0), position),

			new Move(m3N(0.0), m3E(53.0), position),
			new Move(m3N(-50.0), m3E(0.0), position),
			new Move(m3N(0.0), m3E(-50.0), position),//Gets out of first hoop

			new Move(m3N(147.0), m3E(0.0), position),

			new Move(m3N(0.0), m3E(-63.0), position),
			new Move(m3N(-55.0), m3E(0.0), position),
			new Move(m3N(0.0), m3E(127), position),

			new Move(m3N(105.0), m3E(0.0), position),//
			new Move(m3N(0.0), m3E(-47.0), position),
			new Move(1, heading),
			new Move(m3N(-290.0), m3E(0.0), position)
		)
		);
	}


	private static SequentialCommandGroup slalomPath(Subsystem position) {
		///The file in
		///C:\Users\robokm\Desktop\Local2020\2020\_StarDestroyer\STARDESTROYER_1.8_(03-5-21)-Before
		///contains the last used optimised code for the slalom path.

		//Robot will start with the center 16.5in behind the ende as close to the finish zone as posssalbe ~ mostly still correct
		return (new SequentialCommandGroup(
			new Move(m3N(33.5), m3E(0), position),
			new Move(m3N(0), m3E(-(48)), position),

			new Move(m3N(155), m3E(0), position),

			new Move(m3N(0), m3E(70), position),//

			new Move(m3N(40), m3E(0), position),
			new Move(m3N(0), m3E(-50), position),
			new Move(m3N(-50), m3E(0), position),
			new Move(m3N(0), m3E(70), position),

			new Move(m3N(-151), m3E(0), position),

			new Move(m3N(0), m3E(-(69)), position),
			new Move(m3N(-41), m3E(0), position)
		)
		);
	}

	private static SequentialCommandGroup bouncePath(Subsystem position) {
		boolean i = false;
		if(i) {
			return (new SequentialCommandGroup(
				new Move(m3N(40.0), m3E(0.0), position),//center is 20 inches out
				new Move(m3N(10), m3E(-35), position),//Hits first star
				new Move(m3N(30), m3E(-115.0), position),//25 past 60 mark verticaly
				new Move(m3N(30.0), m3E(0.0), position),
				new Move(m3N(15.0), m3E(0.0), position),
				new Move(m3N(15), m3E(-115.0), position),//Hits the second star
				new Move(m3N(15), m3E(115.0), position),
				new Move(m3N(30.0), m3E(0.0), position),
				new Move(m3N(15), m3E(-115.0), position),//Hits the third star
				new Move(m3N(10), m3E(15), position),//Hits first star
				new Move(m3N(40.0), m3E(0.0), position)//center is ~20 inches out
			)
			);
		} else {
			return (new SequentialCommandGroup(
				new Move(m3N(55.0), m3E(0.0), position),
				new Move(m3N(0.0), m3E(-55.0), position),//Hits first cone
				new Move(m3N(0.0), m3E(59.0), position),//Out by D3
				new Move(m3N(27.0), m3E(0.0), position),
				new Move(m3N(0.0), m3E(54.0), position),//Below D5
				new Move(m3N(55.0), m3E(0.0), position),
				new Move(m3N(0.0), m3E(-107.0), position),//Hits the second spot
				new Move(m3N(0.0), m3E(108.0), position),
				new Move(m3N(80.0), m3E(0.0), position),//Second fill left moveup
				new Move(m3N(0.0), m3E(-108.0), position),//Should hit thrid location
				new Move(m3N(0.0), m3E(63.0), position),//Move to in frount of the finish zone
				new Move(m3N(36.0), m3E(0.0), position)
			)
			);

		}
	}

	@Override
	public void execute() {
	}

	// @Override
	// public void initialize() {
	// }

	public enum AutoNavRoute {
		BARREL_RACE,
		SLALOM,
		BOUNCE
	}

}