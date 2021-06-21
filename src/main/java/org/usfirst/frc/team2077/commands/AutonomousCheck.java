// This file, as far as I can tell is only being worked on by me, therefore, currently, this is the file that you want to replace the old versions
package org.usfirst.frc.team2077.commands;

import java.util.Arrays;

import org.usfirst.frc.team2077.commands.AutoNavRoutes.AutoNavRoute;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonomousCheck extends SequentialCommandGroup{

    private boolean runAuto;
    private boolean galacticSearch;
    private boolean a;
    private boolean red;
    private boolean barrelRacing = false;
    private boolean slalom = false;
    private boolean bounce = false;

    public void initialize(){
    }


    @Override
    public void execute() {
        runAuto = SmartDashboard.getBoolean("Run Autonomous", false);

        if (runAuto) {
            new Move(2, 0).schedule(false);
            done = true;
        }
//        galacticSearch = SmartDashboard.getBoolean("Galactic Search", false);
//        a = SmartDashboard.getBoolean("A", false);
//        red = SmartDashboard.getBoolean("Red", false);
//
//        if (!barrelRacing && SmartDashboard.getBoolean("Barrel Racing", false)) {
//            SmartDashboard.putBoolean("Slalom", false);
//            SmartDashboard.putBoolean("Bounce", false);
//        } else if (!slalom && SmartDashboard.getBoolean("Slalom", false)) {
//            SmartDashboard.putBoolean("Barrel Racing", false);
//            SmartDashboard.putBoolean("Bounce", false);
//        } else if (!bounce && SmartDashboard.getBoolean("Bounce", false)) {
//            SmartDashboard.putBoolean("Barrel Racing", false);
//            SmartDashboard.putBoolean("Slalom", false);
//        }
//        barrelRacing = SmartDashboard.getBoolean("Barrel Racing", false);
//        slalom = SmartDashboard.getBoolean("Slalom", false);
//        bounce = SmartDashboard.getBoolean("Bounce", false);
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
//            end(false);
//        }
    }

    private boolean done = false;
    
    public void end(boolean interrupted) {
        super.end(interrupted);
        done = true;
    }
    public boolean isFinished() {
        return done;
    }

    public static void main(String[] args) {
        (new AutonomousCheck()).initialize();
    }
}