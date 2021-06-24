/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.math;

import org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection;

import java.util.EnumMap;

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;

public class Position extends EnumMap<VelocityDirection, Double> {

    public Position() {
        this(new double[]{0, 0, 0});
    }

    public Position(Position from) {
        this(new double[]{from.get(NORTH), from.get(EAST), from.get(ROTATION)});
    }

    public Position(double[] position) {
        super(VelocityDirection.class);
        put(NORTH, position[NORTH.ordinal()]);
        put(EAST, position[EAST.ordinal()]);
        put(ROTATION, position[ROTATION.ordinal()]);
    }

    public void moveAbsolute(double north, double east, double rotation) {
        compute(NORTH, (k, val) -> val + north);
        compute(EAST, (k, v) -> v + east);
        compute(ROTATION, (k, v) -> v + rotation);
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
        double direction = Math.toRadians(get(ROTATION)) + directionStraight + directionCurve;
        moveAbsolute(distance * Math.cos(direction), distance * Math.sin(direction), rotation);
    }

    public EnumMap<VelocityDirection, Double> distanceAbsolute(Position origin) {
        EnumMap<VelocityDirection, Double> distanceTo = this.clone();

        distanceTo.compute(NORTH, (k, v) -> v - origin.getOrDefault(NORTH, 0d));
        distanceTo.compute(EAST, (k, v) -> v - origin.getOrDefault(EAST, 0d));
        distanceTo.compute(ROTATION, (k, v) -> v - origin.getOrDefault(ROTATION, 0d));
        
        return distanceTo;
    }


    public EnumMap<VelocityDirection, Double> distanceRelative(Position origin) {
        EnumMap<VelocityDirection, Double> absolute = distanceAbsolute(origin);
        double rotation = absolute.get(ROTATION);
        double distance = Math.sqrt(absolute.get(NORTH)*absolute.get(NORTH) + absolute.get(EAST)*absolute.get(EAST)); // straight line
        double direction = Math.atan2(absolute.get(EAST), absolute.get(NORTH)) - Math.toRadians(origin.get(ROTATION));
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

        EnumMap<VelocityDirection, Double> distanceTo = new EnumMap<>(VelocityDirection.class);

        distanceTo.put(NORTH, distance * Math.cos(directionStraight));
        distanceTo.put(EAST, distance * Math.sin(directionStraight));
        distanceTo.put(ROTATION, rotation);

        return distanceTo;
    }

    public void set(double north, double east, double heading) {
        put(NORTH, north);
        put(EAST, east);
        setHeading(heading);
    }

    public void setHeading(double heading) {
        put(ROTATION, heading);
    }

    public double[] get() {
        return new double[] {get(NORTH), get(EAST), get(ROTATION)};
    }

    public Position copy() {
        return new Position(this);
    }

    @Override
    public String toString() {
        return "N:" + Math.round(get(NORTH)*10.)/10. + "in E:" + Math.round(get(EAST)*10.)/10. + "in A:" + Math.round(get(ROTATION) * 10.) / 10. + "deg";
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
