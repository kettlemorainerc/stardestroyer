/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.math;

public class Position {

    protected double north_ = 0;
    protected double east_ = 0;
    protected double heading_ = 0;

    public Position() {
    }

    public Position(double[] position) {
        north_ = position[0];
        east_ = position[1];
        heading_ = position[2];
    }

    public void moveAbsolute(double north, double east, double rotation) {
        north_ += north;
        east_ += east;
        heading_ += rotation;
    }

    public void moveRelative(double north, double east, double rotation) {
        double distance = Math.sqrt(north*north + east*east); // along curve
        double directionStraight = Math.atan2(east, north);
        double directionCurve = 0;
        if ( distance != 0 && rotation != 0) {
            double radians = Math.toRadians(rotation);
            double radius = distance / radians;
            double ahead = radius * Math.sin(radians);
            double side = radius - radius * Math.cos(radians);
            directionCurve = Math.atan2(side, ahead);
            distance = Math.sqrt(side*side + ahead*ahead);
        }
        double direction = Math.toRadians(heading_) + directionStraight + directionCurve;
        moveAbsolute(distance * Math.cos(direction), distance * Math.sin(direction), rotation);
    }

    public double[] distanceAbsolute(Position origin) {
        double[] o = origin.get();
        return new double[] {north_-o[0], east_-o[1], heading_-o[2]};
    }


    public double[] distanceRelative(Position origin) {
        double[] absolute = distanceAbsolute(origin);
        double rotation = absolute[2];
        double distance = Math.sqrt(absolute[0]*absolute[0] + absolute[1]*absolute[1]); // straight line
        double direction = Math.atan2(absolute[1], absolute[0]) - Math.toRadians(origin.heading_);
        double directionCurve = 0;
        if ( distance != 0 && rotation != 0) {
            double radians = Math.toRadians(rotation);
            double radius = distance/2/Math.sin(radians/2);
            double ahead = radius * Math.sin(radians);
            double side = radius - radius * Math.cos(radians);
            directionCurve = Math.atan2(side, ahead);
            distance = radius * radians;
        }
        double directionStraight = direction - directionCurve;
        return new double[] {distance*Math.cos(directionStraight), distance*Math.sin(directionStraight), rotation};
    }

    public void set(double north, double east, double heading) {
        north_ = north;
        east_ = east;
        heading_ = heading;
    }

    public double[] get() {
        return new double[] {north_, east_, heading_};
    }

    @Override
    public String toString() {
        return "N:" + Math.round(north_*10.)/10. + "in E:" + Math.round(east_*10.)/10. + "in A:" + Math.round(heading_*10.)/10. +"deg";
    }

    private static String toString(double[] doubleArray) {
        StringBuffer sb = new StringBuffer();
        for (Double d : doubleArray) {
            sb.append(Math.round(d * 10.) / 10.);
            sb.append(" ");
        }
        return sb.toString();
    }

    /**
     * Test code. May be run locally in VSCode.
     */
    public static void main(String[] argv) {
        Position p = new Position();
        Position pp;
        
        p = new Position();
        System.out.println();
        System.out.println(p);

        p = new Position();
        pp = new Position(p.get());
        p.moveRelative(12, 12, 0);
        System.out.println();
        System.out.println("moveRelative(12, 12, 0)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));

        p = new Position();
        pp = new Position(p.get());
        p.moveRelative(0, 0, 90);
        System.out.println();
        System.out.println("moveRelative(0, 0, 90)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));

        p = new Position();
        pp = new Position(p.get());
        p.moveRelative(12, 0, 90);
        System.out.println();
        System.out.println("moveRelative(12, 0, 90)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));

        p = new Position();
        pp = new Position(p.get());
        p.moveRelative(24, 24, 180);
        System.out.println();
        System.out.println("moveRelative(24, 24, 180)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));
        p.moveRelative(0, 10, 0);
        System.out.println("moveRelative(0, 10, 0)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));
        p.moveRelative(0, 10, 0);
        System.out.println("moveRelative(0, 10, 0)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));
        p.moveRelative(0, 10, 0);
        System.out.println("moveRelative(0, 10, 0)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));
        p.moveRelative(0, 10, 0);
        System.out.println("moveRelative(0, 10, 0)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));

        p = new Position();
        pp = new Position(p.get());
        p.moveRelative(12, 0, 0);
        System.out.println();
        System.out.println("moveRelative(12, 0, 0)");
        System.out.println(p);
        System.out.println(toString(p.distanceAbsolute(pp)) + " " + toString(p.distanceRelative(pp)));
       

    }
}
