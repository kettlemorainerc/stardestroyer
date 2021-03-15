package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoNavRoutes extends SequentialCommandGroup {
	///---Mapping---///
	final private static double NORTH_MULTIPLIER = 1.0;
	final private static double EAST_MULTIPLIER = 1.0;
	final private static double ROTATION_MULTIPLIER = 1.0;
///---Mapping---///

//TODO: Calabrate

	public AutoNavRoutes(AutoNavRoute path) {
		switch (path) {
			case BARREL_RACE:
				barrelRace().schedule();
				break;
			case SLALOM:
				slalomPath().schedule();
				break;
			case BOUNCE:
				bouncePath().schedule();
				break;
			default:
				System.out.println("\n   [WARNING]: Else reached for AutoNavRoutes - '26893223421231'\n");//Random string of numbers, so it can be searched in the code
		}
		// if(pathNumber_ == 0){//barrelRace
		//         barrelRace().schedule();
		// }else if(pathNumber_ == 1){//slalomPath
		//         slalomPath().schedule();
		// }else if(pathNumber_ == 2){//bouncePath
		//         bouncePath().schedule();
		// }else{
		//     System.out.println("\n   [WARNING]: Else reached for AutoNavRoutes - '26893223421231'\n");//Random string of numbers, so it can be searched in the code
		// }
	}

	//Callabration values needed.
	private static double m3N(double north_) {//move3N
		return north_ * NORTH_MULTIPLIER;
	}

	private static double m3E(double east_) {
		return east_ * EAST_MULTIPLIER;
	}
//Callabration values needed.

	private static double m3R(double rotation_) {
		return rotation_ * ROTATION_MULTIPLIER;
	}
    private static SequentialCommandGroup barrelRaceOrig() {
        double _unit = 21.5;
        return (new SequentialCommandGroup(
            // new Move(m3N(0.0),m3E(0.0)), //North, East, Rotation
            new Move(m3N(16.5+90.0),m3E(0.0)),
            new Move(m3N(_unit),m3E(_unit)),
            new Move(m3N(-_unit),m3E(_unit)),
            new Move(m3N(-_unit),m3E(_unit)),
            new Move(m3N(_unit),m3E(-_unit)),
            new Move(m3R(-29.05)),
            new Move(m3N(102.956),m3E(0.0)),
            new Move(m3N(0.0),m3E(-_unit)),
            new Move(m3N(-_unit),m3E(0.0)),
            new Move(m3N(94.22),m3E(_unit)),

            new Move(m3N(_unit),m3E(_unit)),
            new Move(m3N(_unit),m3E(-_unit)),
            new Move(m3N(0.0),m3E(_unit)),

            new Move(m3N(300.0),m3E(300.0)) //North, East, Rotation
        // )).schedule();
            )
        );
    }

    private static SequentialCommandGroup barrelRace() {
        return (new SequentialCommandGroup(
            new Move(m3N(135.0),m3E(0.0)),

            new Move(m3N(0.0),m3E(53.0)),
            new Move(m3N(-50.0),m3E(0.0)),
            new Move(m3N(0.0),m3E(-50.0)),//Gets out of first hoop
            // new Move(m3N(0.0),m3E(0.0),m3R(-0.25)),
            // new Move(m3N(0.0),m3E(0.0),m3R(-0.125)),//

            // new Move(m3N(0.0),m3E(0.0),m3R(-5.0)),
            // new Move(m3N(0.0),m3E(0.0),m3R(5.0)),
                // new Move(-0.15)
                // new Move(-1.0),
                // new Move(-0.09),
            // new Move(m3R(5.0)),
            new Move(m3N(147.0),m3E(0.0)),

            new Move(m3N(0.0),m3E(-63.0)),
            new Move(m3N(-55.0),m3E(0.0)),
            new Move(m3N(0.0),m3E(127)),

            // new Move(m3N(90.0),m3E(135.0)),
            new Move(m3N(105.0),m3E(0.0)),//
            // new Move(m3N(70.0),m3E(0.0)),//
            // new Move(m3N(0.0),m3E(80.0)),
            // new Move(m3N(100.0),m3E(58.0)),
            // new Move(m3N(10.0),m3E(0.0)),
            new Move(m3N(0.0),m3E(-47.0)),
            new Move(1),
            new Move(m3N(-290.0),m3E(0.0))


            // new Move(m3N(0.01),m3E(0.01))
            // new Move(m3N(0.0),m3E(300.0))
            // )).schedule();
            )
        );
    }


    private static SequentialCommandGroup slalomPath() {
        ///The file in 
        ///C:\Users\robokm\Desktop\Local2020\2020\_StarDestroyer\STARDESTROYER_1.8_(03-5-21)-Before
        ///contains the last used optimised code for the slalom path.

        //Robot will start with the center 16.5in behind the ende as close to the finish zone as posssalbe
        // return (new SequentialCommandGroup(
        //     new Move(m3N(33),m3E(0)),
        //     new Move(m3N(60),m3E(-(22+0+22))),
        //     new Move(m3N(120),m3E(0.0)),
        //     new Move(m3N(60),m3E(22+16+22)),
        //     new Move(m3N((22+16+22)),m3E(-22+16+22)),
        //     new Move(m3N(-(22+16+22)),m3E(-22+16+22)),
        //     new Move(m3N(-(22+16+22)),m3E(-22+16+22)),
        //     new Move(m3N(-120),m3E(0.0)),
        //     new Move(m3N(-60),m3E(-(22+0+22))),
        //     new Move(m3N(-33),m3E(0))
        //     // )).schedule();
        //     )
        // );
        return (new SequentialCommandGroup(
            new Move(m3N(33.5),m3E(0)),
            new Move(m3N(0),m3E(-(48))),
            // new Move(m3N(60),m3E(0)),//
            // new Move(m3N(120),m3E(0.0)),
            // new Move(m3N(60),m3E(0)),
            // new Move(m3N(0),m3E(22+16+22)),//
            new Move(m3N(155),m3E(0)),

            new Move(m3N(0),m3E(70)),//

            new Move(m3N(40),m3E(0)),
            new Move(m3N(0),m3E(-50)),
            new Move(m3N(-50),m3E(0)),
            new Move(m3N(0),m3E(70)),

            new Move(m3N(-151),m3E(0)),

            new Move(m3N(0),m3E(-(69))),
            new Move(m3N(-41),m3E(0))
            
            // new Move(m3N((22+16+22)),m3E(-22+16+22)),
            // new Move(m3N(-(22+16+22)),m3E(-22+16+22)),
            // new Move(m3N(-(22+16+22)),m3E(-22+16+22)),

            // new Move(m3N(-120),m3E(0.0)),
            // new Move(m3N(0),m3E(-(22+0+22))),
            // new Move(m3N(-60),m3E(0)),//
            // new Move(m3N(-33),m3E(0))
            // )).schedule();
            )
        );
    }
  
    private static SequentialCommandGroup bouncePath() {
        boolean i = false;
        if(i){
        return (new SequentialCommandGroup(
            new Move(m3N(40.0),m3E(0.0)),//center is 20 inches out
            new Move(m3N(10),m3E(-35)),//Hits first star
            new Move(m3N(30),m3E(-115.0)),//25 past 60 mark verticaly
            new Move(m3N(30.0),m3E(0.0)),
            new Move(m3N(15.0),m3E(0.0)),
            new Move(m3N(15),m3E(-115.0)),//Hits the second star
            new Move(m3N(15),m3E(115.0)),
            new Move(m3N(30.0),m3E(0.0)),
            new Move(m3N(15),m3E(-115.0)),//Hits the third star
            new Move(m3N(10),m3E(15)),//Hits first star
            new Move(m3N(40.0),m3E(0.0))//center is ~20 inches out
            // new Move(m3N(0),m3E(0))
            // )).schedule();
            )
        );
        }else{
        return (new SequentialCommandGroup(
            new Move(m3N(55.0),m3E(0.0)),
            new Move(m3N(0.0),m3E(-55.0)),//Hits first cone
            new Move(m3N(0.0),m3E(59.0)),//Out by D3
            new Move(m3N(27.0),m3E(0.0)),
            new Move(m3N(0.0),m3E(54.0)),//Below D5
            new Move(m3N(55.0),m3E(0.0)),
            new Move(m3N(0.0),m3E(-107.0)),//Hits the second spot
            new Move(m3N(0.0),m3E(108.0)),
            new Move(m3N(80.0),m3E(0.0)),//Second fill left moveup
            new Move(m3N(0.0),m3E(-108.0)),//Should hit thrid location
            new Move(m3N(0.0),m3E(63.0)),//Move to in frount of the finish zone
            new Move(m3N(36.0),m3E(0.0))


            // new Move(m3N(0.001),m3E(0.001))
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