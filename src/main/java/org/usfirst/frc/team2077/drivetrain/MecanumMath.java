package org.usfirst.frc.team2077.drivetrain;

/***
 * An implementation of the mecanum drivetrain inverse and forward kinematics described in
 * <a href="http://www.chiefdelphi.com/uploads/default/original/3X/9/3/937da7cbd006480f2b47eb9ee7bd8567b8f22dd9.pdf">
 * <i>Kinematic Analysis of Four-Wheel Mecanum Vehicle</i></a>.
 * <p>
 * The core matrix algebra is implemented in the static methods:
 * <dl>
 *  <dt>{@link #inverse(double[],double[][],double)}</dt>
 *  <dd><p style="margin-left: 40px">Implements the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b>,
 *  which calculates the the individual wheel motions necessary to produce a specified robot motion.
 *  This calculation is the central function of a basic drive control program to convert user input to motor control.</p></dd>
 *  <dt>{@link #forward(double[],double[][],double)}</dt>
 *  <dd><p style="margin-left: 40px">Implements the forward kinematic equation <b>[V] = [F][&#x03A9;](r)</b>,
 *  which calculates the robot motion to be expected from a set of individual wheel motions.
 *  This calculation is not generally necessary for basic robot control, but may be useful for more advanced operations.
 *  Unlike the inverse equation, the forward calculation represents an overdetermined system,
 *  where most combinations of wheel motions are inconsistent. Inconsistent wheel motions in practice mean
 *  wheel slippage, motor stalling, and generally erratic behavior, the more inconsistent the worse.
 *  This forward calculation produces a least-squares best fit.</p></dd>
 *  <dt>{@link #createInverseMatrix(double,double)}, {@link #createInverseMatrix(double,double,double[])}</dt>
 *  <dd><p style="margin-left: 40px">Convenience methods for initializing the inverse kinematic matrix <b>[R]</b>.</p></dd>
 *  <dt>{@link #createForwardMatrix(double,double)}</dt>
 *  <dd><p style="margin-left: 40px">Convenience method for initializing the forward kinematic matrix <b>[F]</b>.</p></dd>
 * </dl>
 * <p>
 * Instance objects wrap the core static methods with specific robot geometry and optional unit
 * conversion factors. Robot geometry, including dimensions, wheel size, and center of rotation, is set in the
 * constructor, and the <b>[R]</b> and <b>[F]</b> matrices are internally managed. Application code may then
 * call the simpler {@link #inverse(double[])} and {@link #forward(double[])} methods with the current robot motion (<b>[V]</b>)
 * or wheel motion (<b>[&#x03A9;]</b>) vectors. Some of the constructors also take conversion factors to automatically
 * convert input and output values for these methods from and to other units more convenient to the calling code. 
 * <div style="font-size: smaller">
 * Note: This implementation departs somewhat from the notational conventions used in the paper above:
 * <ul>
 * <li>"North" and "East" are used instead of X and Y.</li>
 * <li>Orientation is mirror-imaged: Y (East) is positive-right, and rotation is positive-clockwise.
 * <li>Wheel numbers are also mirror imaged, so the matrix math itself is unchanged.</li>
 * </ul>
 * </div>
 * @author 2077
 */
public final class MecanumMath {

    /***
     * Solve the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b>.
     * @param V Robot motion vector <b>[V]</b>: [north/south translation (distance), east/west translation, rotation (radians)].
     * @param R A 4x3 inverse kinematic matrix <b>[R]</b>. See {@link #createInverseMatrix}.
     * @param r wheel radius, in the length units used to construct <b>[R]</b>.
     * @return The wheel angular velocity vector <b>[&#x03A9;]</b>: {NE, SE, SW, NW} in radians.
     */
    public static double[] inverse(double[] V, double[][] R, double r) {
         double[] O = new double[4];
         for (int i = 0; i < 4; i++) {
             for (int j = 0; j < 3; j++) {
                 O[i] += (V[j] * R[i][j]) / r;
             }
         }
         return O;
     }
     
    /***
     * Solve the forward kinematic equation <b>[V] = [F][&#x03A9;](r)</b>.
     * Where <b>[&#x03A9;]</b> has internal inconsistencies a best-fit value is returned.
     * @param O A wheel angular motion vector <b>[&#x03A9;]</b>: [NE, SE, SW, NW] in radians.
     * @param F A 3x4 forward kinematic matrix <b>[F]</b>. See {@link #createForwardMatrix}.
     * @param r Wheel radius, in the length units used to construct <b>[F]</b>.
     * @return Th <b>[V]</b>: [north/south translation (distance), east/west translation, rotation (radians)].
     */
    public static double[] forward(double[] O, double[][] F, double r) {
         double[] V = new double[3];
         for (int i = 0; i < 3; i++) {
             for (int j = 0; j < 4; j++) {
                 V[i] += (O[j] * F[i][j]) * r;
             }
         }
         return V;
     }
     
    /**
     * Construct the inverse matrix <b>[R]</b> for a rectangular robot with an arbitrary center of rotation.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param rotationCenter {N,E} coordinates of the robot's center of rotation relative to its geometric center.
     * @return A 4x3 inverse kinematic matrix <b>[R]</b> for use by {@link #inverse}.
     */
    public static double[][] createInverseMatrix(double length, double width, double[] rotationCenter) {

        double[][] xy = {
            { length/2 - rotationCenter[0], width/2 - rotationCenter[1]},
            {-length/2 - rotationCenter[0], width/2 - rotationCenter[1]},
            {-length/2 - rotationCenter[0],-width/2 - rotationCenter[1]},
            { length/2 - rotationCenter[0],-width/2 - rotationCenter[1]}
        };
        return new double[][] {
            {1,-1,-xy[0][0]-xy[0][1]},
            {1, 1, xy[1][0]-xy[1][1]},
            {1,-1,-xy[2][0]-xy[2][1]},
            {1, 1, xy[3][0]-xy[3][1]}
        };
    }
     
    /**
     * Construct the inverse matrix <b>[R]</b> for a rectangular robot whose center of rotation is its geometric center.
     * This is a convenience function wrapping {@link #createInverseMatrix(double,double,double[])}.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @return A 4x3 inverse kinematic matrix <b>[R]</b> for use by {@link #inverse}.
     */
    public static double[][] createInverseMatrix(double length, double width) {
        //return createInverseMatrix(length, width, new double[] {0,0});

        //should give same as above
        double K = Math.abs(length/2) + Math.abs(width/2);
        return new double[][] {
            {1,-1,-K},
            {1, 1,-K},
            {1,-1, K},
            {1, 1, K}
        };

    }
     
    
    /**
     * Construct the forward matrix <b>[F]</b> for a given inverse matrix <b>[R]</b>.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @return A 3x4 forward kinematic matrix <b>[F]</b> for use by {@link #forward}.
     */
    public static double[][] createForwardMatrix(double length, double width) {

        double K = length/2 + width/2;
        return new double[][] {
            { 1/4.,    1/4.,    1/4.,    1/4.   },
            {-1/4.,    1/4.,   -1/4.,    1/4.   },
            {-1/(4*K),-1/(4*K), 1/(4*K), 1/(4*K)}
        };
    }

    /**
     * Construct the forward matrix <b>[F]</b> for a rectangular robot.
     * @param inverse Inverse kinematic matrix <b>[R]</b>.
     * @return A 3x4 forward kinematic matrix <b>[F]</b> for use by {@link #forward}.
     */
    public static double[][] createForwardMatrix(double[][] r) {

        double[][] rT = transpose(r);
        double[][] f = multiply(invert3x3(multiply(rT, r)), rT);
        System.out.println( "INVERSE" + toString(r) );
        System.out.println( "FORWARD" + (massage(f)) );
        return f;
    }

    private final double[][] reverseMatrix_;
    private final double[][] forwardMatrix_;
    private final double length_;
    private final double width_;
    private final double wheelRadius_;
    private final double wheelSpeedFactor_;
    private final double robotSpeedFactor_;
    private final double rotationSpeedFactor_;
    
    /**
     * This is a convenience constructor wrapping {@link #MecanumMath(double,double,double,double,double,double)}.
     * Calls to {@link #inverse(double[])} or {@link #forward(double[])} will calculate translation motions
     * using the same length units as length, width, and wheel radius, and robot and wheel rotations in radians.
     * This constructor is equivalent to <code>MecanumMath(length,width,wheelRadius,1,1,1)</code>.
     * @param length Distance between wheel contact points in the N/S direction, in any length unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param wheelRadius Radius of each wheel, in the same unit.
     */
    public MecanumMath(double length, double width, double wheelRadius) {
        this(length, width, wheelRadius, 1, 1, 1);
    }

    /**
     * Initialize a robot-specific context for inverse and forward kinematics solutions,
     * with internal conversion to and from the default units, with the robot's center of rotation at its geometric center.
     * This constructor is equivalent to
     * <code>MecanumMath(length,width,wheelRadius,wheelSpeedFactor,robotSpeedFactor,rotationSpeedFactor,{0,0})</code>.
     * @param length Distance between wheel contact points in the N/S direction, in any distance unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param wheelRadius Radius of each wheel, in the same unit.
     * @param wheelSpeedFactor Multiplier to convert wheel rotation in radians to user rotation units.
     * For example, if wheelSpeedFactor == wheelRadius, wheel velocities supplied to {@link #forward(double[])} and
     * returned from {@link #inverse(double[])} will be in user distance units.
     * If wheelSpeedFactor == 60/(2*Math.PI), wheel velocity user units will be in revolutions per minute.
     * @param robotSpeedFactor Multiplier to convert robot translation motions in length distance to user units.
     * While it's possible to use a conversion factor here, generally using robotSpeedFactor == 1
     * will be most sensible, keeping supplied and returned robot translation motions in length units.
     * @param rotationSpeedFactor Multiplier to convert robot rotations in radians to user rotation units.
     * For example, if rotationSpeedFactor = 180/Math.PI, robot rotations passed to {@link #inverse(double[])}
     * and returned from {@link #forward(double[])} will be in degrees.
     */
	public MecanumMath(double length, double width, double wheelRadius, double wheelSpeedFactor, double robotSpeedFactor, double rotationSpeedFactor) {
        this(length, width, wheelRadius, wheelSpeedFactor, robotSpeedFactor, rotationSpeedFactor, new double[] {0, 0});
	}

    /**
     * Initialize a robot-specific context for inverse and forward kinematics solutions,
     * with the center of rotation at a specified point.
     * This constructor is equivalent to
     * <code>MecanumMath(length,width,wheelRadius,1,1,1,rotationCenter)</code>.
     * @param length Distance between wheel contact points in the N/S direction, in any distance unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param wheelRadius Radius of each wheel, in the same unit.
     * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
     */
	public MecanumMath(double length, double width, double wheelRadius, double[] rotationCenter) {
        this(length, width, wheelRadius, 1, 1, 1, rotationCenter);
	}
   

    /**
     * Initialize a robot-specific context for inverse and forward kinematics solutions,
     * with internal conversion to and from the default units.
     * @param length Distance between wheel contact points in the N/S direction, in any distance unit.
     * @param width Distance between wheel contact points in the E/W direction, in the same unit.
     * @param wheelRadius Radius of each wheel, in the same unit.
     * @param wheelSpeedFactor Multiplier to convert wheel rotation in radians to user rotation units.
     * For example, if wheelSpeedFactor == wheelRadius, wheel velocities supplied to {@link #forward(double[])} and
     * returned from {@link #inverse(double[])} will be in user distance units.
     * If wheelSpeedFactor == 60/(2*Math.PI), wheel velocity user units will be in revolutions per minute.
     * @param robotSpeedFactor Multiplier to convert robot translation motions in length distance to user units.
     * While it's possible to use a conversion factor here, generally using robotSpeedFactor == 1
     * will be most sensible, keeping supplied and returned robot translation motions in length units.
     * @param rotationSpeedFactor Multiplier to convert robot rotations in radians to user rotation units.
     * For example, if rotationSpeedFactor = 180/Math.PI, robot rotations passed to {@link #inverse(double[])}
     * and returned from {@link #forward(double[])} will be in degrees.
     * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
     */
	public MecanumMath(double length, double width, double wheelRadius, double wheelSpeedFactor, double robotSpeedFactor, double rotationSpeedFactor, double[] rotationCenter) {
		length_ = length;
		width_ = width;
		reverseMatrix_ = createInverseMatrix(length, width, rotationCenter);
		//forwardMatrix_ = createForwardMatrix(length, width);
		forwardMatrix_ = createForwardMatrix(reverseMatrix_);
		wheelRadius_ = wheelRadius;
		wheelSpeedFactor_ = wheelSpeedFactor;
		robotSpeedFactor_ = robotSpeedFactor;
		rotationSpeedFactor_ = rotationSpeedFactor;
	}
   
    /***
     * Solve the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b>.
     * This method wraps the static {@link #inverse(double[],double[][],double)} method, passing the internal
     * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
     * values from and to user units if conversion factors were specified.
     * @param userV <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
     * @return The wheel speed vector <b>[&#x03A9;]</b>: [NE, SE, SW, NW] in user units.
     */
    public final double[] inverse(double[] userV) {
    	return inverse(userV, new double[] {0, 0});
    }

    /***
     * @Deprecated
     * Solve the inverse kinematic equation <b>[&#x03A9;] = (1/r)[R][V]</b> for rotation about an arbitrary point.
     * This method wraps the static {@link #inverse(double[],double[][],double)} method, passing the internal
     * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
     * values from and to user units if conversion factors were specified.
     * @param userV <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
     * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
     * @return The wheel speed vector <b>[&#x03A9;]</b>: [NE, SE, SW, NW] in user units.
     */
    public final double[] inverse(double[] userV, double[] rotationCenter) {
        double[] v = { fromUserRobotSpeed(userV[0]), fromUserRobotSpeed(userV[1]), fromUserRotationSpeed(userV[2]) };
    	double[][] reverseMatrix = rotationCenter[0]==0 && rotationCenter[1]==0 ? reverseMatrix_ : createInverseMatrix(length_, width_, rotationCenter);
        double[] o = inverse(v, reverseMatrix, wheelRadius_);
        return new double[] { toUserWheelSpeed(o[0]), toUserWheelSpeed(o[1]), toUserWheelSpeed(o[2]), toUserWheelSpeed(o[3]) };
    }

    /***
     * Solve the forward kinematic equation <b>[V] = [F][&#x03A9;](r)</b>.
     * This method wraps the static {@link #forward(double[],double[][],double)} method, passing the internal
     * <b>[F]</b> matrix and wheel radius initialized in the constructor, and converting input and output
     * values from and to user units if conversion factors were specified.
     * @param userO A wheel speed vector <b>[&#x03A9;]</b>: [NE, SE, SW, NW] in user units.
     * @return [3] robot motion velocities [north/south translation, east/west translation, rotation] in user units.
     */
    public final double[] forward(double[] userO) {
        double[] o = { fromUserWheelSpeed(userO[0]), fromUserWheelSpeed(userO[1]), fromUserWheelSpeed(userO[2]), fromUserWheelSpeed(userO[3]) };
        double[] v = forward(o, forwardMatrix_, wheelRadius_);
        return new double[] { toUserRobotSpeed(v[0]), toUserRobotSpeed(v[1]), toUserRotationSpeed(v[2]) };
    }
    
    /***
     * Convert a wheel rotation speed from user units to radians
     * by dividing the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * @param userWheelSpeed An individual wheel speed in user units
     * as input to {@link #forward(double[])}.
     * @return Wheel rotational velicity in radians
     * as expected by {@link #forward(double[],double[][],double)}.
     */
    private double fromUserWheelSpeed(double userWheelSpeed) {
        return userWheelSpeed / wheelSpeedFactor_;
    }

    /***
     * Convert a robot translation speed from user units to distance
     * (where distance is in the length/width/radius units passed to the constructor)
     * by dividing the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * @param userRobotSpeed A N/S or E/W robot translation velocity component in user units
     * as input to {@link #inverse(double[])}.
     * @return Velocity component in length distance
     * as expected by {@link #inverse(double[],double[][],double)}.
     */
    private double fromUserRobotSpeed(double userRobotSpeed) {
        return userRobotSpeed / robotSpeedFactor_;
    }

    /***
     * Convert a robot rotation speed from user units to radians
     * by dividing the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * @param userRotationSpeed A robot rotational velocity in user units
     * as input to {@link #inverse(double[])}.
     * @return Rotational velocity (radians)
     * as expected by {@link #inverse(double[],double[][],double)}.
     */
    private double fromUserRotationSpeed(double userRotationSpeed) {
        return userRotationSpeed / rotationSpeedFactor_;
    }

    /***
     * Convert a wheel rotation speed from radians to user units
     * by multiplying the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * @param radiansPerSecond An individual wheel rotational velicity in radians
     * as returned by {@link #inverse(double[],double[][],double)}.
     * @return Speed in user units as returned by {@link #inverse(double[])}.
     */
    private double toUserWheelSpeed(double radiansPerSecond) {
        return radiansPerSecond * wheelSpeedFactor_;
    }

    /***
     * Convert a robot translation speed from distance
     * (where distance is in the length/width/radius units passed to the constructor) to user units
     * by multiplying the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * @param unitsPerSecond A N/S or E/W robot translation velocity component in in distance
     * as returned by {@link #forward(double[],double[][],double)}.
     * @return Speed in user units as returned by {@link #forward(double[])}.
     */
    private double toUserRobotSpeed(double unitsPerSecond) {
        return unitsPerSecond * robotSpeedFactor_;
    }

    /***
     * Convert a robot rotation speed from radians to user units
     * by multiplying the input value by the conversion factor passed to the
     * constructor {@link #MecanumMath(double,double,double,double[],double,double,double)}.
     * @param radiansPerSecond A robot rotational velocity in radians
     * as returned by {@link #forward(double[],double[][],double)}.
     * @return Speed in user units as returned by {@link #forward(double[])}.
     */
    private double toUserRotationSpeed(double radiansPerSecond) {
        return radiansPerSecond * rotationSpeedFactor_;
    }

    /**
     * Compute robot-relative <b>[V]</b> ({north/south translation, east/west translation, rotation}) motion
     * ncessary to reach location <b>[D]</b> ({north/south position, east/west position, heading})
     * from initial location {0, 0, 0}, where translation and rotation speeds are constant or proportional
     * throughout.
     * @param userD <b>[D]</b>: total displacement {north/south translation, east/west translation, heading}, in user units.
     * @return [3] robot-relative motion {north/south translation, east/west translation, rotation} in user units.
     */
    public final double[] travel(double[] userD) {

        double[] d = { fromUserRobotSpeed(userD[0]), fromUserRobotSpeed(userD[1]), fromUserRotationSpeed(userD[2]) };

        double headingChange = d[2];
        while ( headingChange < -Math.PI ) headingChange += 2*Math.PI;
        while ( headingChange > Math.PI ) headingChange -= 2*Math.PI;

        double displacementDistance = Math.sqrt(d[0]*d[0] + d[1]*d[1]);
        double displacementAngle = Math.atan2(d[1], d[0]);
        // portion of displacement angle due to curvature
        double curveAngle = Math.atan2(Math.sin(headingChange), 1 - Math.cos(headingChange));
        double travelDistance = headingChange == 0 ? displacementDistance : (displacementDistance * headingChange / (2 * Math.sin(headingChange / 2)));
        double travelAngle = displacementAngle - curveAngle;
        double travelN = travelDistance * Math.cos(travelAngle);
        double travelE = travelDistance * Math.sin(travelAngle);

        return new double[] { toUserRobotSpeed(travelN), toUserRobotSpeed(travelE), toUserRotationSpeed(headingChange) };
    }
    
   
    /***
     * Compute robot-relative <b>[D]</b>  ({north/south position, east/west position, heading}) displacement
     * for a motion <b>[V]</b> ({north/south translation, east/west translation, rotation})
     * from initial location {0, 0, 0}, where translation and rotation speeds are constant or proportional
     * throughout.
     * @param user <b>[V]</b>: {north/south translation, east/west translation, rotation}, in user units.
     * @return [3] robot displacement {north/east translation, heading} in user units.
     */
    public final double[] displacement(double[] userV) {

        double[] v = { fromUserRobotSpeed(userV[0]), fromUserRobotSpeed(userV[1]), fromUserRotationSpeed(userV[2]) };

        double headingChange = v[2];
        while ( headingChange < -Math.PI ) headingChange += 2*Math.PI;
        while ( headingChange > Math.PI ) headingChange -= 2*Math.PI;

        double travelDistance = Math.sqrt(v[0]*v[0] + v[1]*v[1]);
        double travelAngle = Math.atan2(v[1], v[0]);
        // portion of displacement angle due to curvature
        double curveAngle = Math.atan2(Math.sin(headingChange), 1 - Math.cos(headingChange));
        double displacementDistance = headingChange == 0 ? travelDistance : (travelDistance * (2 * Math.sin(headingChange / 2)) / headingChange);
        double displacementAngle = curveAngle + travelAngle;
        double displacementN = displacementDistance * Math.cos(displacementAngle);
        double displacementE = displacementDistance * Math.sin(displacementAngle);

        return new double[] { toUserRobotSpeed(displacementN), toUserRobotSpeed(displacementE), toUserRotationSpeed(headingChange) };
    }


    private static String toString(double[][] m) {
        StringBuffer sb = new StringBuffer();
        int rows = m.length;
        int cols = m[0].length;
        sb.append("\n{ ");
        for (int r = 0; r < rows; r++) {
            sb.append("\n{ ");
            for (int c = 0; c < cols; c++) {
                sb.append(roundd(m[r][c]));
                sb.append("\t");
            }
            sb.append("}");
        }
        sb.append("\n}");
        return sb.toString();
    }


    private static String toString(double[] v) {
        StringBuffer sb = new StringBuffer();
        int n = v.length;
        sb.append("\n{ ");
        for (int i = 0; i < n; i++) {
            sb.append(roundd(v[i]));
            sb.append("\t");
        }
        sb.append("}");
        return sb.toString();
    }


    private static double[][] massage(double[][] m) {
        int rows = m.length;
        int cols = m[0].length;
        double[][] x = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                x[r][c] = 1 / m[r][c] / 4;
            }
        }
        return x;
    }


    /**
     * Matrix inversion (3x3 only).
     * @param m 3x3 matrix <b>[M]</b>.
     * @return <b>[M]</b><sup>-1</sup>.
     */
    private static double[][] invert3x3(double[][] m) {
        double determinate
            = m[0][0] * m[1][1] * m[2][2]
            + m[0][1] * m[1][2] * m[2][0]
            + m[0][2] * m[1][0] * m[2][1]
            - m[2][0] * m[1][1] * m[0][2]
            - m[2][1] * m[1][2] * m[0][0]
            - m[2][2] * m[1][0] * m[0][1];
        double[][] inverse = {
            {m[1][1] * m[2][2] - m[1][2] * m[2][1], m[0][2] * m[2][1] - m[0][1] * m[2][2], m[0][1] * m[1][2] - m[0][2] * m[1][1]},
            {m[1][2] * m[2][0] - m[1][0] * m[2][2], m[0][0] * m[2][2] - m[0][2] * m[2][0], m[0][2] * m[1][0] - m[0][0] * m[1][2]},
            {m[1][0] * m[2][1] - m[1][1] * m[2][0], m[0][1] * m[2][0] - m[0][0] * m[2][1], m[0][0] * m[1][1] - m[0][1] * m[1][0]}
        };
        if (determinate != 0) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    inverse[i][j] /= determinate;
                }
            }
        }
        return inverse;
    }


    /**
     * Matrix transposition.
     * @param m Matrix <b>[M]</b>.
     * @return <b>[M]</b>'.
     */
    private static double[][] transpose(double[][] m) {
        int rows = m[0].length;
        int cols = m.length;
        double[][] t = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                t[r][c] = m[c][r];
            }
        }
        return t;
    }


    /**
     * Matrix multiplication.
     * @param a Matrix <b>[A]</b>.
     * @param b Matrix <b>[B]</b>.
     * @return <b>[A]</b>x<b>[B]</b>.
     */
    public static double[][] multiply(double[][] a, double[][] b) {
        int n = b.length;
        int rows = a.length;
        int cols = b[0].length;
        double[][] p = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                for (int i = 0; i < n; i++) {
                    p[r][c] += a[r][i] * b[i][c];
                }
            }
        }
        return p;
    }


    public static double roundd(double d) {
        return Math.round(1000.*d)/1000.;
    }


    public static void main000000000000(String[] argv) {
    
        double testRobotLength = 3;
        double testRobotWidth = 5;
        double testRobotWheelRadius = 1;

        MecanumMath math = new MecanumMath(testRobotLength, testRobotWidth, testRobotWheelRadius);

        double _90 = Math.PI/2;
        double[][] dd = {
            {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 0, _90}, {1, 0, _90}, {0, 1, _90}, {1, 1, _90},
            {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 0, -_90}, {1, 0, -_90}, {0, 1, _90}, {1, 1, -_90},
            {0, 0, 0}, {-1, 0, 0}, {-1, 1, 0}, {0, 0, _90}, {-1, 0, _90}, {0, 1, _90}, {-1, 1, _90},
            {0, 0, 0}, {-1, 0, 0}, {-1, 1, 0}, {0, 0, -_90}, {-1, 0, -_90}, {0, 1, _90}, {-1, 1, -_90},
            {0, 0, 0}, {1, 0, 0}, {1, -1, 0}, {0, 0, _90}, {1, 0, _90}, {0, -1, _90}, {1, -1, _90},
            {0, 0, 0}, {1, 0, 0}, {1, -1, 0}, {0, 0, -_90}, {1, 0, -_90}, {0, -1, _90}, {1, -1, -_90},
            {0, 0, 0}, {-1, 0, 0}, {-1, -1, 0}, {0, 0, _90}, {-1, 0, _90}, {0, -1, _90}, {-1, -1, _90},
            {0, 0, 0}, {-1, 0, 0}, {-1, -1, 0}, {0, 0, -_90}, {-1, 0, -_90}, {0, -1, _90}, {-1, -1, -_90} };

        for ( double[] d : dd ) {
            double[] m = math.travel(d);
            double[] d2 = math.displacement(m);
            boolean pass = Math.round(d2[0]*1000) == Math.round(d[0]*1000)
                && Math.round(d2[1]*1000) == Math.round(d[1]*1000)
                && Math.round(d2[2]*1000) == Math.round(d[2]*1000);
            System.out.println( "" + roundd(d[0]) + "/" + roundd(d[1]) + "/" + roundd(d[2])
                + "\t" + roundd(m[0]) + "/" + roundd(m[1]) + "/" + roundd(m[2])
                + "\t" + roundd(d2[0]) + "/" + roundd(d2[1]) + "/" + roundd(d2[2]) + " " + pass );

        }



 }


    
    public static void mainAA(String[] argv) {
    
        // can a series of random motions be backed out using cumulative wheel encoder readings?
        
        double testRobotLength = 1;
        double testRobotWidth = 1;
        double testRobotWheelRadius = 1;

        final double[] location = {0, 0, 0}; // {n/s, e/e, heading}

        final double[] odometer = {0, 0, 0, 0}; // {ne, se, sw, nw}

        MecanumMath math = new MecanumMath(testRobotLength, testRobotWidth, testRobotWheelRadius);

        class MM {

            void move(double ns, double ew, double rotation) {
                double[] deltaO = math.inverse( new double[] {ns, ew, rotation} );
                for (int i = 0; i < 4; i++) {
                    odometer[i] += deltaO[i];
                }

                double pathLength = Math.sqrt( ns*ns + ew*ew );
                double initialAngle = location[2] + Math.atan2( ew, ns );
                double[] deltaNE = {0,0}; // relative to an initialAngle of 0 in world coordinates
                if (rotation == 0) {
                    deltaNE[0] = pathLength;
                    deltaNE[1] = 0;
                }
                else {
                    double radius = pathLength / rotation;
                    deltaNE[0] = radius * Math.sin(rotation);
                    deltaNE[1] = radius * (1 - Math.cos(rotation));
                }
                double[] deltaNE2 = {deltaNE[0]*Math.cos(-initialAngle) + deltaNE[1]*Math.sin(-initialAngle),
                                     deltaNE[1]*Math.cos(-initialAngle) - deltaNE[0]*Math.sin(-initialAngle)};

                location[0] += deltaNE2[0];
                location[1] += deltaNE2[1];
                location[2] += rotation;
            }
            void home() {
                double[] ns_ew_rotation = math.forward(new double[] {-odometer[0], -odometer[1], -odometer[2], -odometer[3]});
                move(ns_ew_rotation[0], ns_ew_rotation[1], ns_ew_rotation[2]);
            }
            double round(int significantDigits, double value) {
                return Math.abs(value) < .000001 ? 0. : new java.math.BigDecimal( value ).round( new java.math.MathContext( significantDigits ) ).doubleValue();
            }
            double roundDP(int decimalPlaces, double value) {
                return Math.round((10^decimalPlaces) * value) / (10^decimalPlaces);
            }
            String string(double[] array) {
                String s = new String();
                for (double a : array) {
                    s += "" + round(3, a) + "\t";
                }
                return s;
            }
        };
        MM mm = new MM();

        for (int i = 0; i <= 8; i++) {
            location[0] = location[1] = location[2] = 0;
            odometer[0] = odometer[1] = odometer[2] = odometer[3] = 0;
            mm.move(1, 0, i * Math.PI / 4);
            System.out.println(mm.string(odometer) + "\t" + mm.string(location));
            mm.home();
            // all the following should be zero
            System.out.println(mm.string(odometer) + "\t" + mm.string(location));
            System.out.println();
        }

        System.out.println();

        for (int i = 0; i <= 8; i++) {
            location[0] = location[1] = location[2] = 0;
            odometer[0] = odometer[1] = odometer[2] = odometer[3] = 0;
            mm.move(.5, 0, i * Math.PI / 8);
            mm.move(.5, 0, i * Math.PI / 8);
            System.out.println(mm.string(odometer) + "\t" + mm.string(location));
            mm.home();
            // all the following should be zero
            System.out.println(mm.string(odometer) + "\t" + mm.string(location));
            System.out.println();
        }

        System.out.println();

        for (int i = 0; i <= 8; i++) {
            location[0] = location[1] = location[2] = 0;
            odometer[0] = odometer[1] = odometer[2] = odometer[3] = 0;
            mm.move(0, 1, i * Math.PI / 4);
            System.out.println(mm.string(odometer) + "\t" + mm.string(location));
            mm.home();
            // all the following should be zero
            System.out.println(mm.string(odometer) + "\t" + mm.string(location));
            System.out.println();
        }

        System.out.println();

        location[0] = location[1] = location[2] = 0;
        odometer[0] = odometer[1] = odometer[2] = odometer[3] = 0;
        java.util.Random random = new java.util.Random();
        for (int i = 0; i < 1000; i ++) {
            mm.move(random.nextInt(100), random.nextInt(100), random.nextInt(100));
        }
        System.out.println(mm.string(odometer) + "\t" + mm.string(location));
        mm.home();
        // all the following should be zero
        System.out.println(mm.string(odometer) + "\t" + mm.string(location));
        System.out.println();
    }
    
    public static void mainZZ(String[] argv) {
        
        double testRobotLength = 1;
        double testRobotWidth = 1;
        double testRobotWheelRadius = 1;
        
        double[][] R = createInverseMatrix(testRobotLength, testRobotWidth);
        double[][] F = createForwardMatrix(testRobotLength, testRobotWidth);
        double r = testRobotWheelRadius;

        for (int angle = 0; angle < 360; angle += 15) {
            double[] V = { Math.cos( Math.toRadians( angle ) ), Math.sin( Math.toRadians( angle ) ), 0 };
            double[] O = MecanumMath.inverse( V, R, r );
            double maxAbs = 0;
            for (int i = 0; i < O.length; i++) {
                maxAbs = Math.max( maxAbs,  Math.abs( O[i] ) );
            }
            for (int i = 0; i < O.length; i++) {
                O[i] /= Math.max( 1, maxAbs );
            }
            V = MecanumMath.forward( O, F, r );
            int[] o = { (int)Math.round(O[0]*100), (int)Math.round(O[1]*100), (int)Math.round(O[2]*100), (int)Math.round(O[3]*100) };
            int[] v = { (int)Math.round(V[0]*100), (int)Math.round(V[1]*100), (int)Math.round(V[2]*100) };
            System.out.println( "Angle:\t" + angle
                                + "\t\tWheels:\t" + o[0] + "\t" + o[1] + "\t" + o[2] + "\t" + o[3]
                                + "\t\tN/E/R:\t" + v[0] + "\t" + v[1] + "\t" + v[2] );
        }
        double[] V = { 0, 0, 1 };
        double[] O = MecanumMath.inverse( V, R, r );
        int[] o = { (int)Math.round(O[0]*100), (int)Math.round(O[1]*100), (int)Math.round(O[2]*100), (int)Math.round(O[3]*100) };
        System.out.println( "Rotation 1" + "\t\tWheels:" + o[0] + "\t" + o[1] + "\t" + o[2] + "\t" + o[3] );
    }
    
    /**
     * Test code.
     * @param argv Not used.
     */
    public static void main(String[] argv) {
    	
        // default test robot dimensions are in inches from "pizzabot" test drivetrain
    	double testRobotLength = 17.5;
    	double testRobotWidth = 18.125;
    	double testRobotWheelRadius = 3.0;
    	
        //double[][] R = createInverseMatrix(testRobotLength, testRobotWidth);
        //double[][] F = createForwardMatrix(testRobotLength, testRobotWidth);
        double[][] R = createInverseMatrix(testRobotLength, testRobotWidth, new double[] {testRobotLength/2, 0});
        double[][] F = createForwardMatrix(R);
        double r = testRobotWheelRadius;

        double[] V;
        double[] O;
        MecanumMath mecanum;

        System.out.println("static tests with test robot and default units");
        
        System.out.println();
        System.out.println("L:" + testRobotLength + " W:" + testRobotWidth + " R:" + testRobotWheelRadius );
        System.out.println("V:{Math.PI*2*r, 0, 0}");
        V = new double[] {Math.PI*2*r, 0, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
       
        System.out.println();
        System.out.println("L:" + testRobotLength + " W:" + testRobotWidth + " R:" + testRobotWheelRadius );
        System.out.println("V:{Math.PI*2*r/2, Math.PI*2*r/2, 0}");
        V = new double[] {Math.PI*2*r/2, Math.PI*2*r/2, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
     
        System.out.println();
        System.out.println("V:{0, Math.PI*2*r, 0}");
        V = new double[] {0, Math.PI*2*r, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
      
        System.out.println();
        System.out.println("V:{0, 0, Math.PI/2}");
        V = new double[] {0, 0, Math.PI/2};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = inverse(V, R, r);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = forward(O, F, r);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        System.out.println();

        
        // test robot, inches/second, inches/second, degrees/second
        mecanum = new MecanumMath(testRobotLength, testRobotWidth, testRobotWheelRadius, testRobotWheelRadius, 1, 180/Math.PI, new double[]{0,0});
        System.out.println();
        System.out.println( "test robot, inches/second, inches/second, degrees/second" );

        System.out.println();
        V = new double[] {120, 0, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
       
        System.out.println();
        V = new double[] {120/2, 120/2, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
     
        System.out.println();
        V = new double[] {0, 120, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
      
        System.out.println();
        V = new double[] {0, 0, 90};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        System.out.println();

        
        // test robot, percentage of a given maximum speed, inches/second, degrees/second
        double topSpeedInchesPerSecond = 120;
        mecanum = new MecanumMath(testRobotLength, testRobotWidth, testRobotWheelRadius, 100*testRobotWheelRadius/topSpeedInchesPerSecond, 100/topSpeedInchesPerSecond, 180/Math.PI);
        System.out.println();
        System.out.println( "test robot, percentage of top speed, inches/second, degrees/second" );

        System.out.println();
        V = new double[] {100, 0, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
       
        System.out.println();
        V = new double[] {50, 50, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        System.out.println();
        V = new double[] {0, 100, 0};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        System.out.println();
        V = new double[] {0, 0, 360};
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        //System.out.println();
        //V = new double[] {0, 0, 360};
        //double[] COR = new double[] {testRobotLength/2, -testRobotWidth/2};
        //System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2] + " Center of Rotation:{" + COR[0] + ", " + COR[1] + "}");
        //O = mecanum.inverse(V, COR);
        //System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        //V = mecanum.forward(O);
        //System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        
        System.out.println();
        O = new double[] {100, 100, -100, -100};
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        
        System.out.println();
        System.out.println("inconsistent wheel velocities");
        
        System.out.println();
        O = new double[] {100, -100, -100, 100};
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        V = mecanum.forward(O);
        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
        O = mecanum.inverse(V);
        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
        
        System.out.println();
       
        
        
       
        
//        // test robot, percentage of top speed, inches/second, inches/second of wheel contact point about robot center
//        //double topSpeedInchesPerSecond = 120;
//        double topSpeedRadiandPerSecond = topSpeedInchesPerSecond/(2*Math.PI*testRobotWheelRadius)*(2*Math.PI);
//        mecanum = new MecanumMath(testRobotLength, testRobotWidth, testRobotWheelRadius, topSpeedRadiandPerSecond, SpeedUnit.PERCENT_OF_MAX, SpeedUnit.PERCENT_OF_MAX, SpeedUnit.RADIANS_PER_SECOND);
//        System.out.println();
//        System.out.println( "zzzzzzzzzzzzzzzzzzz test robot, percentage of top speed, inches/second, degrees/second" );
//
//        System.out.println();
//        V = new double[] {topSpeedInchesPerSecond, 0, 0};
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//        O = mecanum.inverse(V);
//        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
//        V = mecanum.forward(O);
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//       
//        System.out.println();
//        V = new double[] {topSpeedInchesPerSecond/2, topSpeedInchesPerSecond/2, 0};
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//        O = mecanum.inverse(V);
//        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
//        V = mecanum.forward(O);
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//     
//        System.out.println();
//        V = new double[] {0, topSpeedInchesPerSecond, 0};
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//        O = mecanum.inverse(V);
//        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
//        V = mecanum.forward(O);
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//      
//        System.out.println();
//        V = new double[] {0, 0, topSpeedInchesPerSecond/Math.sqrt(2)};
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//        O = mecanum.inverse(V);
//        System.out.println("Wheels: {" + O[0] + ", " + O[1]  + ", " + O[2]  + ", " + O[3] + "}");
//        V = mecanum.forward(O);
//        System.out.println("Translation:{" + V[0] + ", " + V[1] + "} Rotation:" + V[2]);
//        
//        System.out.println();
       
       
        
        
        
        
        // test to prove the validity of interpolating wheel speeds between beginning and ending robot motion velocity vectors
        System.out.println();
        System.out.println( "interpolation tests" );
        System.out.println();
        // Talon SRX speed units here are clicks/100ms, where clicks are for the AMT103 encoders as configured on the test drivetrain
        // wheel speed conversion is to radians/second
		double wheelSpeedFactor = 8192 / 10 / (2 * Math.PI); // Talon SRX speed units here are clicks/100ms, this converts to radians/second
		// robot speed units here are inches/second, so no conversion from inch length units is necessary
		double robotSpeedFactor = 1;
		// robot rotation speed units here are inches/second of the wheel contact points about the robot center, this converts to radians/second
		double circumference = Math.PI * Math.sqrt(testRobotLength*testRobotLength + testRobotWidth*testRobotWidth);
		double rotationSpeedFactor = circumference / (2 * Math.PI); 
		mecanum = new MecanumMath(testRobotLength, testRobotWidth, testRobotWheelRadius, wheelSpeedFactor, robotSpeedFactor, rotationSpeedFactor);
		// some random motion velocity vectors
		double[][] vRandom = new double[10][];
		java.util.Random random = new java.util.Random();
		for (int i = 0; i < 10; i++) {
			vRandom[i] = new double[] {2*random.nextInt(36)-36, 2*random.nextInt(36)-36, 2*random.nextInt(36)-36};
		}
		for (int i = 0; i < vRandom.length - 1; i++) {
			double[] vI = vRandom[i]; // beginning robot velocity vector
			double[] wI = mecanum.inverse(vI); // beginning wheel speed vector
			for (int j = i + 1; j < vRandom.length - 1; j++) {
				double[] vJ = vRandom[j]; // ending robot velocity vector
				double[] wJ = mecanum.inverse(vJ); // ending wheel speed vector
				// do some interpolations and check for consistency
				for (int k = 1; k < 10 - 1; k++) {
					// interpolated robot velocity vector
					double[] vIJ = { (k * vI[0] + (10 - k) * vJ[0]) / 10, (k * vI[1] + (10 - k) * vJ[1]) / 10, (k * vI[2] + (10 - k) * vJ[2]) / 10 };
					// wheel speed vector from interpolated robot velocity
					double[] wIJ0 = mecanum.inverse(vIJ);
					// interpolated wheel speed vector
					double[] wIJ1 = { (k * wI[0] + (10 - k) * wJ[0]) / 10, (k * wI[1] + (10 - k) * wJ[1]) / 10, (k * wI[2] + (10 - k) * wJ[2]) / 10, (k * wI[3] + (10 - k) * wJ[3]) / 10 };
					// do wheel speeds from interpolated velocities match interpolated wheel speeds?
					System.out.println("" + i + " " + j + " " + k
							 + " "+ (Math.round(wIJ0[0] - wIJ1[0]) == 0.0) + " " + (Math.round(wIJ0[1] - wIJ1[1]) == 0.0)
							 + " "+ (Math.round(wIJ0[2] - wIJ1[2]) == 0.0) + " " + (Math.round(wIJ0[3] - wIJ1[3]) == 0.0));
				}
			}
		}
    }
}
