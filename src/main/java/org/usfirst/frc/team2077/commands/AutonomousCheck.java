// This file, as far as I can tell is only being worked on by me, therefore, currently, this is the file that you want to replace the old versions
package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.usfirst.frc.team2077.commands.AutoNavRoutes.AutoNavRoute;


public class AutonomousCheck extends SequentialCommandGroup{
    public static final String A_FLAG = "A",
        RED_FLAG = "Red",
        SLALOM_FLAG = "Slalom",
        BOUNCE_FLAG = "Bounce",
        BARREL_RACE_FLAG = "Barrel Racing",
        GALACTIC_SEARCH_FLAG = "Galactic Search",
        AUTO_FLAG = "Run Autonomous";

    private boolean runAuto;
    private boolean galacticSearch;
    private boolean a;
    private boolean red;
    private boolean barrelRacing = false;
    private boolean slalom = false;
    private boolean bounce = false;

    public void initialize() {
//        SmartDashboard.putBoolean(AUTO_FLAG, false);
//        SmartDashboard.putBoolean(GALACTIC_SEARCH_FLAG, false);
//        SmartDashboard.putBoolean(A_FLAG, false);
//        SmartDashboard.putBoolean(RED_FLAG, false);
//        SmartDashboard.putBoolean(SLALOM_FLAG, false);
//        SmartDashboard.putBoolean(BOUNCE_FLAG, false);
//        SmartDashboard.putBoolean(BARREL_RACE_FLAG, false);
    }


    @Override
    public void execute() {
        runAuto = SmartDashboard.getBoolean(AUTO_FLAG, false);
//
        if (runAuto) {
            new Move(60, 0).schedule(false);
            done = true;
        }
//        galacticSearch = SmartDashboard.getBoolean(GALACTIC_SEARCH_FLAG, false);
//        a = SmartDashboard.getBoolean(A_FLAG, false);
//        red = SmartDashboard.getBoolean(RED_FLAG, false);
//
//        if (!barrelRacing && SmartDashboard.getBoolean(BARREL_RACE_FLAG, false)) {
//            SmartDashboard.putBoolean(SLALOM_FLAG, false);
//            SmartDashboard.putBoolean(BOUNCE_FLAG, false);
//        } else if (!slalom && SmartDashboard.getBoolean(SLALOM_FLAG, false)) {
//            SmartDashboard.putBoolean(BARREL_RACE_FLAG, false);
//            SmartDashboard.putBoolean(BOUNCE_FLAG, false);
//        } else if (!bounce && SmartDashboard.getBoolean(BOUNCE_FLAG, false)) {
//            SmartDashboard.putBoolean(BARREL_RACE_FLAG, false);
//            SmartDashboard.putBoolean(SLALOM_FLAG, false);
//        }
//        barrelRacing = SmartDashboard.getBoolean(BARREL_RACE_FLAG, false);
//        slalom = SmartDashboard.getBoolean(SLALOM_FLAG, false);
//        bounce = SmartDashboard.getBoolean(BOUNCE_FLAG, false);
//
//        if (runAuto) {
//            if (galacticSearch) {
//                // System.out.println("GALACTIC SEARCH TIME _______________");
//                (new GalacticSearch(a, red)).schedule();
//            } else {
//                if (barrelRacing) {
//                    (new AutoNavRoutes(AutoNavRoute.BARREL_RACE)).schedule();
//                } else if (slalom) {
//                    (new AutoNavRoutes(AutoNavRoute.SLALOM)).schedule();
//                } else if (bounce) {
//                    (new AutoNavRoutes(AutoNavRoute.BOUNCE)).schedule();
//                }
//            }
//            done = true;
//        }
    }

    private boolean done = false;
    
    public boolean isFinished() {
        return done;
    }

    public static void main(String[] args) {
        (new AutonomousCheck()).initialize();
    }
}