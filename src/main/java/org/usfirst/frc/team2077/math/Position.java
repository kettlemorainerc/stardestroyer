/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.math;

import org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction;

import java.util.EnumMap;

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction.*;

public class Position extends EnumMap<Direction, Double> {

    public Position() {
        this(new double[]{0, 0, 0});
    }

    public Position(Position from) {
        this(new double[]{from.get(NORTH), from.get(EAST), from.get(CLOCKWISE)});
    }

    public Position(double[] position) {
        super(Direction.class);
        put(NORTH, position[NORTH.ordinal()]);
        put(EAST, position[EAST.ordinal()]);
        put(CLOCKWISE, position[CLOCKWISE.ordinal()]);
    }

    public void moveAbsolute(double north, double east, double rotation) {
        compute(NORTH, (k, val) -> val + north);
        compute(EAST, (k, v) -> v + east);
        compute(CLOCKWISE, (k, v) -> v + rotation);
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
        double direction = Math.toRadians(get(CLOCKWISE)) + directionStraight + directionCurve;
        moveAbsolute(distance * Math.cos(direction), distance * Math.sin(direction), rotation);
    }

    public EnumMap<Direction, Double> distanceAbsolute(Position origin) {
        EnumMap<Direction, Double> distanceTo = this.clone();

        distanceTo.compute(NORTH, (k, v) -> v - origin.getOrDefault(NORTH, 0d));
        distanceTo.compute(EAST, (k, v) -> v - origin.getOrDefault(EAST, 0d));
        distanceTo.compute(CLOCKWISE, (k, v) -> v - origin.getOrDefault(CLOCKWISE, 0d));
        
        return distanceTo;
    }


    public EnumMap<Direction, Double> distanceRelative(Position origin) {
        EnumMap<Direction, Double> absolute = distanceAbsolute(origin);
        double rotation = absolute.get(CLOCKWISE);
        double distance = Math.sqrt(absolute.get(NORTH)*absolute.get(NORTH) + absolute.get(EAST)*absolute.get(EAST)); // straight line
        double direction = Math.atan2(absolute.get(EAST), absolute.get(NORTH)) - Math.toRadians(origin.get(CLOCKWISE));
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

        EnumMap<Direction, Double> distanceTo = new EnumMap<>(Direction.class);

        distanceTo.put(NORTH, distance * Math.cos(directionStraight));
        distanceTo.put(EAST, distance * Math.cos(directionStraight));
        distanceTo.put(CLOCKWISE, rotation);

        return distanceTo;
    }

    public void set(double north, double east, double heading) {
        put(NORTH, north);
        put(EAST, east);
        put(CLOCKWISE, heading);
    }

    public double[] get() {
        return new double[] {get(NORTH), get(EAST), get(CLOCKWISE)};
    }

    public Position copy() {
        return new Position(this);
    }

    @Override
    public String toString() {
        return "N:" + Math.round(get(NORTH)*10.)/10. + "in E:" + Math.round(get(EAST)*10.)/10. + "in A:" + Math.round(get(CLOCKWISE)*10.)/10. +"deg";
    }

    private static String toString(double[] doubleArray) {
        StringBuffer sb = new StringBuffer();
        for (Double d : doubleArray) {
            sb.append(Math.round(d * 10.) / 10.);
            sb.append(" ");
        }
        return sb.toString();
    }
}
