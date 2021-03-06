package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.commands.Move2;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import static org.usfirst.frc.team2077.Robot.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc.team2077.subsystems.Launcher;//NEW

public class AutoNavRoutes extends SequentialCommandGroup {
    ///---Mapping---///
    final private static double NORTH_MULTIPLIER = 1.0;
    final private static double EAST_MULTIPLIER = 1.0;
    final private static double ROTATION_MULTIPLIER = 1.0;
///---Mapping---///

//TODO: Calabrate

//Callabration values needed.
private static double m3N(double north_){//move3N
    return north_*NORTH_MULTIPLIER;
}
private static double m3E(double east_){
    return east_*EAST_MULTIPLIER;
}
private static double m3R(double rotation_){
    return rotation_*ROTATION_MULTIPLIER;
}
//Callabration values needed.

    private static SequentialCommandGroup barrelRace() {
        double _unit = 21.5;
        return (new SequentialCommandGroup(
            // new Move2(m3N(0.0),m3E(0.0)), //North, East, Rotation
            new Move2(m3N(16.5+90.0),m3E(0.0)),
            new Move2(m3N(_unit),m3E(_unit)),
            new Move2(m3N(-_unit),m3E(_unit)),
            new Move2(m3N(-_unit),m3E(_unit)),
            new Move2(m3N(_unit),m3E(-_unit)),
            new Move2(m3R(-29.05)),
            new Move2(m3N(102.956),m3E(0.0)),
            new Move2(m3N(0.0),m3E(-_unit)),
            new Move2(m3N(-_unit),m3E(0.0)),
            new Move2(m3N(94.22),m3E(_unit)),

            new Move2(m3N(_unit),m3E(_unit)),
            new Move2(m3N(_unit),m3E(-_unit)),
            new Move2(m3N(0.0),m3E(_unit)),

            new Move2(m3N(300.0),m3E(300.0)) //North, East, Rotation
        // )).schedule();
            )
        );
    }

    private static SequentialCommandGroup slalomPath() {
        //Robot will start with the center 16.5in behind the ende as close to the finish zone as posssalbe
        // return (new SequentialCommandGroup(
        //     new Move2(m3N(33),m3E(0)),
        //     new Move2(m3N(60),m3E(-(22+0+22))),
        //     new Move2(m3N(120),m3E(0.0)),
        //     new Move2(m3N(60),m3E(22+16+22)),
        //     new Move2(m3N((22+16+22)),m3E(-22+16+22)),
        //     new Move2(m3N(-(22+16+22)),m3E(-22+16+22)),
        //     new Move2(m3N(-(22+16+22)),m3E(-22+16+22)),
        //     new Move2(m3N(-120),m3E(0.0)),
        //     new Move2(m3N(-60),m3E(-(22+0+22))),
        //     new Move2(m3N(-33),m3E(0))
        //     // )).schedule();
        //     )
        // );
        return (new SequentialCommandGroup(
            new Move2(m3N(33.5),m3E(0)),
            new Move2(m3N(0),m3E(-(48))),
            // new Move2(m3N(60),m3E(0)),//
            // new Move2(m3N(120),m3E(0.0)),
            // new Move2(m3N(60),m3E(0)),
            // new Move2(m3N(0),m3E(22+16+22)),//
            new Move2(m3N(155),m3E(0)),

            new Move2(m3N(0),m3E(70)),//

            new Move2(m3N(40),m3E(0)),
            new Move2(m3N(0),m3E(-50)),
            new Move2(m3N(-50),m3E(0)),
            new Move2(m3N(0),m3E(70)),

            new Move2(m3N(-151),m3E(0)),

            new Move2(m3N(0),m3E(-(69))),
            new Move2(m3N(-41),m3E(0))
            
            // new Move2(m3N((22+16+22)),m3E(-22+16+22)),
            // new Move2(m3N(-(22+16+22)),m3E(-22+16+22)),
            // new Move2(m3N(-(22+16+22)),m3E(-22+16+22)),

            // new Move2(m3N(-120),m3E(0.0)),
            // new Move2(m3N(0),m3E(-(22+0+22))),
            // new Move2(m3N(-60),m3E(0)),//
            // new Move2(m3N(-33),m3E(0))
            // )).schedule();
            )
        );
    }
  
    private static SequentialCommandGroup bouncePath() {
        return (new SequentialCommandGroup(
            new Move2(m3N(40.0),m3E(0.0)),//center is 20 inches out
            new Move2(m3N(10),m3E(-35)),//Hits first star
            new Move2(m3N(30),m3E(-115.0)),//25 past 60 mark verticaly
            new Move2(m3N(30.0),m3E(0.0)),
            new Move2(m3N(15.0),m3E(0.0)),
            new Move2(m3N(15),m3E(-115.0)),//Hits the second star
            new Move2(m3N(15),m3E(115.0)),
            new Move2(m3N(30.0),m3E(0.0)),
            new Move2(m3N(15),m3E(-115.0)),//Hits the third star
            new Move2(m3N(10),m3E(35)),//Hits first star
            new Move2(m3N(40.0),m3E(0.0))//center is ~20 inches out
            // new Move2(m3N(0),m3E(0))
            // )).schedule();
            )
        );
        }


    public enum AutoNavRoute {
        BARREL_RACE,
        SLALOM,
        BOUNCE
    }

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

    // @Override
    // public void initialize() {

    // }

    @Override
    public void execute() {
    }

}