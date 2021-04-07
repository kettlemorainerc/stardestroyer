package org.usfirst.frc.team2077.drivetrain;

// I believe the entire purpose of this class is to find [V] or [&Omega;]. AKA the velocity of the actual bot or the velocity of each individual wheel -- David

import java.util.*;

/***
 * An implementation of the mecanum drivetrain inverse and forward kinematics described in
 * <a href="http://www.chiefdelphi.com/uploads/default/original/3X/9/3/937da7cbd006480f2b47eb9ee7bd8567b8f22dd9.pdf">
 * <i>Kinematic Analysis of Four-Wheel Mecanum Vehicle</i></a>.
 * <style>
 *     #terms table { padding: .25rem; border: 1px solid black; border-collapse: collapse; }
 *     #terms td { padding: .25rem; border: 1px solid black; text-align: center; }
 *
 *     #terms dt { border-bottom: 2px solid black; font-size: 1.2em; }
 *     #terms dd { margin: 0; padding: .5em 1em .5em 1.5em; border-left: 2px solid black; }
 *     #terms blockquote { margin: 0; }
 * </style>
 * <dl id=terms>
 *     <dt>[V]</dt>
 *     <dd><blockquote>Velocity matrix of the vehicle at a given moment, essentially determining how the vehicle is moving/has moved
 *     <br>
 *     <b>[V] = {N/S, E/W, Rotational} Velocities</b></blockquote></dd>
 *     <dt>[&Omega;]</dt>
 *     <dd><blockquote>Essentially the {NE, SE, SW, NW} wheel velocities. "<b>Inverse Kinematic</b>"</blockquote></dd>
 * 	   <dt>X<sub>n</sub></dt>
 * 	   <dd><blockquote>N/S point of the wheel</blockquote></dd>
 * 	   <dt>Y<sub>n</sub></dt>
 * 	   <dd><blockquote>E/W point of the wheel</blockquote></dd>
 * 	   <dt>K</dt>
 * 	   <dd><blockquote>
 * 	       <b>abs(X<sub>n</sub>)</b> + <b>abs(Y<sub>n</sub>)</b>
 * 	   </blockquote></dd>
 *     <dt>[R]</dt>
 *     <dd><blockquote>4x3 matrix that allows us to calculate [&Omega;] given [V]
 *         <br>
 *         <br>
 *         <table>
 *             <tr>
 *                 <td>1</td>
 *                 <td>-1</td>
 *                 <td>-K</td>
 *             </tr>
 *             <tr>
 *                 <td>1</td>
 *                 <td>1</td>
 *                 <td>-K</td>
 *             </tr>
 *             <tr>
 *                 <td>1</td>
 *                 <td>-1</td>
 *                 <td>K</td>
 *             </tr>
 *             <tr>
 *                 <td>1</td>
 *                 <td>1</td>
 *                 <td>K</td>
 *             </tr>
 *         </table>
 *     </blockquote>
 *     <br><b>[&Omega;] = (1 / wheelRadius)[R][V]</b></blockquote></dd>
 * 	   <dt>[V<sub>n</sub>]</dt>
 * 	   <dd><blockquote><b>n</b>th wheel's velocity vector.</blockquote></dd>
 * 	   <dt>[&Theta;<sub>n</sub>]</dt>
 * 	   <dd><blockquote>"Anticlockwise angle that the axis of the mecanum roller of wheel <b>n</b>
 * 	   in contact with the floor makes with the <b>X</b> axis"</blockquote></dd>
 * 	   <dt>r</dt>
 * 	   <dd><blockquote>Radius of each wheel</blockquote></dd>
 * 	   <dt>[F]</dt>
 * 	   <dd><blockquote>
 * 	       3x4 matrix allowing to solve for <b>[V]</b> given <b>[&Omega;]</b> and <b>r</b>
 * 	       <br>
 * 	       <b>[F][&Omega;]r = [V]</b>
 *		   <br>
 *		   <br>
 *		   <table summary="" id="forward-matrix">
 *		       <tr>
 *		           <td>1 / 4</td>
 *		           <td>1 / 4</td>
 *		           <td>1 / 4</td>
 *		           <td>1 / 4</td>
 *		       </tr>
 *		       <tr>
 *		           <td>-1 / 4</td>
 *		           <td> 1 / 4</td>
 *		           <td>-1 / 4</td>
 *		           <td> 1 / 4</td>
 *		       </tr>
 *		       <tr>
 *		           <td>-1 / 4K</td>
 *		           <td>-1 / 4K</td>
 *		           <td> 1 / 4K</td>
 *		           <td> 1 / 4K</td>
 *		       </tr>
 *		   </table>
 * 	   </blockquote></dd>
 * </dl>
 * <p>
 * The core matrix algebra is implemented in the static methods:
 * <dl>
 *  <dt>{@link #inverse(EnumMap, double[])}</dt>
 *  <dd><p style="margin-left: 40px">Implements the inverse kinematic equation <b>[&Omega;] = (1/r)[R][V]</b>,
 *  which calculates the the individual wheel motions necessary to produce a specified robot motion.
 *  This calculation is the central function of a basic drive control program to convert user input to motor control.</p></dd>
 *  <dt>{@link #forward(EnumMap)}}</dt>
 *  <dd><p style="margin-left: 40px">Implements the forward kinematic equation <b>[V] = [F][&Omega;](r)</b>,
 *  which calculates the robot motion to be expected from a set of individual wheel motions.
 *  This calculation is not generally necessary for basic robot control, but may be useful for more advanced operations.
 *  Unlike the inverse equation, the forward calculation represents an overdetermined system,
 *  where most combinations of wheel motions are inconsistent. Inconsistent wheel motions in practice mean
 *  wheel slippage, motor stalling, and generally erratic behavior, the more inconsistent the worse.
 *  This forward calculation produces a least-squares best fit.</p></dd>
 *  <dt>{@link #createInverseMatrix(double, double)}, {@link #createInverseMatrix(double, double, double[])}</dt>
 *  <dd><p style="margin-left: 40px">Convenience methods for initializing the inverse kinematic matrix <b>[R]</b>.</p></dd>
 *  <dt>{@link #createForwardMatrix(double, double)}</dt>
 *  <dd><p style="margin-left: 40px">Convenience method for initializing the forward kinematic matrix <b>[F]</b>.</p></dd>
 * </dl>
 * <p>
 * Instance objects wrap the core static methods with specific robot geometry and optional unit
 * conversion factors. Robot geometry, including dimensions, wheel size, and center of rotation, is set in the
 * constructor, and the <b>[R]</b> and <b>[F]</b> matrices are internally managed. Application code may then
 * call the simpler {@link #inverse(EnumMap)} and {@link #forward(EnumMap)} methods with the current robot motion (<b>[V]</b>)
 * or wheel motion (<b>[&Omega;]</b>) vectors. Some of the constructors also take conversion factors to automatically
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
	/**
	 * These are the positions that this class cares about in regards to a motor/wheel/controller/encoder assembly
	 */
	public enum AssemblyPosition {
		NORTH_EAST,
		SOUTH_EAST,
		SOUTH_WEST,
		NORTH_WEST
	}

	// TODO: name this better?
	public enum VelocityDirection {
		NORTH,
		EAST, CLOCKWISE
	}

	/**
	 * Construct the inverse matrix <b>[R]</b> for a rectangular robot with an arbitrary center of rotation.
	 *
	 * @param length         Distance between wheel contact points in the N/S direction, in any length unit.
	 * @param width          Distance between wheel contact points in the E/W direction, in the same unit.
	 * @param rotationCenter {N,E} coordinates of the robot's center of rotation relative to its geometric center.
	 * @return A 4x3 inverse kinematic matrix <b>[R]</b> for use by {@link #inverse}.
	 */
	public static double[][] createInverseMatrix(double length, double width, double[] rotationCenter) {

		double[][] xy = {
			{length / 2 - rotationCenter[0], width / 2 - rotationCenter[1]},
			{-length / 2 - rotationCenter[0], width / 2 - rotationCenter[1]},
			{-length / 2 - rotationCenter[0], -width / 2 - rotationCenter[1]},
			{length / 2 - rotationCenter[0], -width / 2 - rotationCenter[1]}
		};
		return new double[][]{
			{1, -1, -xy[0][0] - xy[0][1]},
			{1, 1, xy[1][0] - xy[1][1]},
			{1, -1, -xy[2][0] - xy[2][1]},
			{1, 1, xy[3][0] - xy[3][1]}
		};
	}

	/**
	 * Construct the inverse matrix <b>[R]</b> for a rectangular robot whose center of rotation is its geometric center.
	 * This is a convenience function wrapping {@link #createInverseMatrix(double, double, double[])}.
	 *
	 * @param length Distance between wheel contact points in the N/S direction, in any length unit.
	 * @param width  Distance between wheel contact points in the E/W direction, in the same unit.
	 * @return A 4x3 inverse kinematic matrix <b>[R]</b> for use by {@link #inverse}.
	 */
	public static double[][] createInverseMatrix(double length, double width) {
		//return createInverseMatrix(length, width, new double[] {0,0});

		//should give same as above
		double K = Math.abs(length / 2) + Math.abs(width / 2);
		return new double[][]{
			{1, -1, -K},
			{1, 1, -K},
			{1, -1, K},
			{1, 1, K}
		};

	}


	/**
	 * Construct the forward matrix <b>[F]</b> for a given inverse matrix <b>[R]</b>.
	 *
	 * @param length Distance between wheel contact points in the N/S direction, in any length unit.
	 * @param width  Distance between wheel contact points in the E/W direction, in the same unit.
	 * @return A 3x4 forward kinematic matrix <b>[F]</b> for use by {@link #forward}.
	 */
	public static double[][] createForwardMatrix(double length, double width) {

		double K = length / 2 + width / 2;
		return new double[][]{
			{1 / 4., 1 / 4., 1 / 4., 1 / 4.},
			{-1 / 4., 1 / 4., -1 / 4., 1 / 4.},
			{-1 / (4 * K), -1 / (4 * K), 1 / (4 * K), 1 / (4 * K)}
		};
	}

	/**
	 * Construct the forward matrix <b>[F]</b> for a rectangular robot.
	 *
	 * Performs <b>(([R]<sup>T</sup>[R])[R]<sup>T</sup>)</b> to calculate <b>[F]</b>
	 *
	 * @param inverseKinematicMatrix <b>[R]</b> - Inverse kinematic matrix 4x3.
	 * @return <b>[F]</b> - 3x4 forward kinematic matrix for use by {@link #forward}.
	 */
	public static double[][] createForwardMatrix(double[][] inverseKinematicMatrix) {

		double[][] rT = transpose(inverseKinematicMatrix); // 3x4
		double[][] f = multiply(invert3x3(multiply(rT, inverseKinematicMatrix)), rT);
		System.out.println("INVERSE" + toString(inverseKinematicMatrix));
		System.out.println("FORWARD" + (massage(f)));
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
	 * This is a convenience constructor wrapping {@link #MecanumMath(double, double, double, double, double, double)}.
	 * Calls to {@link #inverse(EnumMap)} (double[])} or {@link #forward(EnumMap)} will calculate translation motions
	 * using the same length units as length, width, and wheel radius, and robot and wheel rotations in radians.
	 * This constructor is equivalent to <code>MecanumMath(length,width,wheelRadius,1,1,1)</code>.
	 *
	 * @param length      Distance between wheel contact points in the N/S direction, in any length unit.
	 * @param width       Distance between wheel contact points in the E/W direction, in the same unit.
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
	 *
	 * @param length              Distance between wheel contact points in the N/S direction, in any distance unit.
	 * @param width               Distance between wheel contact points in the E/W direction, in the same unit.
	 * @param wheelRadius         Radius of each wheel, in the same unit.
	 * @param wheelSpeedFactor    Multiplier to convert wheel rotation in radians to user rotation units.
	 *                            For example, if wheelSpeedFactor == wheelRadius, wheel velocities supplied to {@link #forward(EnumMap)} and
	 *                            returned from {@link #inverse(EnumMap)} will be in user distance units.
	 *                            If wheelSpeedFactor == 60/(2*Math.PI), wheel velocity user units will be in revolutions per minute.
	 * @param robotSpeedFactor    Multiplier to convert robot translation motions in length distance to user units.
	 *                            While it's possible to use a conversion factor here, generally using robotSpeedFactor == 1
	 *                            will be most sensible, keeping supplied and returned robot translation motions in length units.
	 * @param rotationSpeedFactor Multiplier to convert robot rotations in radians to user rotation units.
	 *                            For example, if rotationSpeedFactor = 180/Math.PI, robot rotations passed to {@link #inverse(EnumMap)}
	 *                            and returned from {@link #forward(EnumMap)} will be in degrees.
	 */
	public MecanumMath(double length, double width, double wheelRadius, double wheelSpeedFactor, double robotSpeedFactor, double rotationSpeedFactor) {
		this(length, width, wheelRadius, wheelSpeedFactor, robotSpeedFactor, rotationSpeedFactor, new double[]{0, 0});
	}

	/**
	 * Initialize a robot-specific context for inverse and forward kinematics solutions,
	 * with the center of rotation at a specified point.
	 * This constructor is equivalent to
	 * <code>MecanumMath(length,width,wheelRadius,1,1,1,rotationCenter)</code>.
	 *
	 * @param length         Distance between wheel contact points in the N/S direction, in any distance unit.
	 * @param width          Distance between wheel contact points in the E/W direction, in the same unit.
	 * @param wheelRadius    Radius of each wheel, in the same unit.
	 * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
	 */
	public MecanumMath(double length, double width, double wheelRadius, double[] rotationCenter) {
		this(length, width, wheelRadius, 1, 1, 1, rotationCenter);
	}


	/**
	 * Initialize a robot-specific context for inverse and forward kinematics solutions,
	 * with internal conversion to and from the default units.
	 *
	 * @param length              Distance between wheel contact points in the N/S direction, in any distance unit.
	 * @param width               Distance between wheel contact points in the E/W direction, in the same unit.
	 * @param wheelRadius         Radius of each wheel, in the same unit.
	 * @param wheelSpeedFactor    Multiplier to convert wheel rotation in radians to user rotation units.
	 *                            For example, if wheelSpeedFactor == wheelRadius, wheel velocities supplied to {@link #forward(EnumMap)} and
	 *                            returned from {@link #inverse(EnumMap)} will be in user distance units.
	 *                            If wheelSpeedFactor == 60/(2*Math.PI), wheel velocity user units will be in revolutions per minute.
	 * @param robotSpeedFactor    Multiplier to convert robot translation motions in length distance to user units.
	 *                            While it's possible to use a conversion factor here, generally using robotSpeedFactor == 1
	 *                            will be most sensible, keeping supplied and returned robot translation motions in length units.
	 * @param rotationSpeedFactor Multiplier to convert robot rotations in radians to user rotation units.
	 *                            For example, if rotationSpeedFactor = 180/Math.PI, robot rotations passed to {@link #inverse(EnumMap)}
	 *                            and returned from {@link #forward(EnumMap)} (double[])} will be in degrees.
	 * @param rotationCenter      {N,E} coordinates of the center of rotation relative to the robot's geometric center.
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
	 * Solve the inverse kinematic equation <b>[&Omega;] = (1/r)[R][V]</b>.
	 * This method wraps the static {@link #inverse(EnumMap, double[])} method, passing the internal
	 * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
	 * values from and to user units if conversion factors were specified.
	 * @param translationMatrix <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
	 * @return The wheel speed vector <b>[&Omega;]</b>: [NE, SE, SW, NW] in user units.
	 */
	public final EnumMap<AssemblyPosition, Double> inverse(EnumMap<VelocityDirection, Double> translationMatrix) {
		return inverse(translationMatrix, new double[]{0, 0});
	}


//	/***
//	 * Solve the inverse kinematic equation <b>[&Omega;] = (1/r)[R][V]</b>.
//	 * @param motionVector Robot motion vector <b>[V]</b>: [north/south translation (distance), east/west translation, rotation (radians)].
//	 * @param inverseKineticMatrix A 4x3 inverse kinematic matrix <b>[R]</b>. See {@link #createInverseMatrix}.
//	 * @param wheelRadius wheel radius, in the length units used to construct <b>[R]</b>.
//	 * @return The wheel angular velocity vector <b>[&Omega;]</b>: {NE, SE, SW, NW} in radians.
//	 */
//	 * This method wraps the static {@link #inverse(double[], double[][], double)} method, passing the internal
	/***
	 * Solve the inverse kinematic equation <b>[&Omega;] = (1/r)[R][V]</b> for rotation about an arbitrary point.
	 * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
	 * values from and to user units if conversion factors were specified.
	 * @param translationMatrix <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
	 * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
	 * @return The wheel speed vector <b>[&Omega;]</b>: [NE, SE, SW, NW] in user units.
	 */
	private EnumMap<AssemblyPosition, Double> inverse(EnumMap<VelocityDirection, Double> translationMatrix, double[] rotationCenter) {
		double[][] inverseKineticMatrix = rotationCenter[0] == 0 && rotationCenter[1] == 0 ?
			reverseMatrix_ :
			createInverseMatrix(length_, width_, rotationCenter);

		EnumMap<AssemblyPosition, Double> wheelSpeedVector = new EnumMap<>(AssemblyPosition.class);
		for(AssemblyPosition position : AssemblyPosition.values()) {
			wheelSpeedVector.put(position, 0d);

			for(VelocityDirection direction : VelocityDirection.values()) {
				wheelSpeedVector.compute(position, (key, val) -> (
					val + (fromUserRobotSpeed(translationMatrix.get(direction)) *
					       inverseKineticMatrix[position.ordinal()][direction.ordinal()]) / wheelRadius_
					));
			}

			wheelSpeedVector.compute(position, (key, val) -> toUserWheelSpeed(val));
		}

		return wheelSpeedVector;
	}

	/***
	 * Solve the forward kinematic equation <b>[V] = [F][&Omega;](r)</b>.
	 * Where <b>[&Omega;]</b> has internal inconsistencies a best-fit value is returned.
	 * @param motionVector A wheel angular motion vector <b>[&Omega;]</b>: [NE, SE, SW, NW] in radians.
	 * @param kinematicMatrix A 3x4 forward kinematic matrix <b>[F]</b>. See {@link #createForwardMatrix}.
	 * @param wheelRadius Wheel radius, in the length units used to construct <b>[F]</b>.
	 * @return Th <b>[V]</b>: [north/south translation (distance), east/west translation, rotation (radians)].
	 */
	/***
	 * Solve the forward kinematic equation <b>[V] = [F][&Omega;](r)</b>.
	 * This method wraps the static {@link #forward(EnumMap)} method, passing the internal
	 * <b>[F]</b> matrix and wheel radius initialized in the constructor, and converting input and output
	 * values from and to user units if conversion factors were specified.
	 * @param wheelSpeeds A wheel speed vector <b>[&Omega;]</b>: [NE, SE, SW, NW] in user units.
	 * @return [3] robot motion velocities [north/south translation, east/west translation, rotation] in user units.
	 */
	public final EnumMap<VelocityDirection, Double> forward(EnumMap<AssemblyPosition, Double> wheelSpeeds) {
		EnumMap<VelocityDirection, Double> translation = new EnumMap<>(VelocityDirection.class);

		for(VelocityDirection direction: VelocityDirection.values()) {
			translation.put(direction, 0d);
			for(AssemblyPosition position : AssemblyPosition.values()) {
				translation.compute(direction, (k, val) -> (
					val + (fromUserWheelSpeed(wheelSpeeds.get(position)) *
						   forwardMatrix_[direction.ordinal()][position.ordinal()] *
						   wheelRadius_)
				));
			}
			translation.compute(direction, (k, val) -> toUserWheelSpeed(val));
		}

		return translation;
	}

	/***
	 * Convert a wheel rotation speed from user units to radians
	 * by dividing the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, double[])}.
	 * @param userWheelSpeed An individual wheel speed in user units
	 * as input to {@link #forward(EnumMap)}.
	 * @return Wheel rotational velicity in radians
	 * as expected by {@link #forward(EnumMap)}.
	 */
	private double fromUserWheelSpeed(double userWheelSpeed) {
		return userWheelSpeed / wheelSpeedFactor_;
	}

	/***
	 * Convert a robot translation speed from user units to distance
	 * (where distance is in the length/width/radius units passed to the constructor)
	 * by dividing the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, double[])}.
	 * @param userRobotSpeed A N/S or E/W robot translation velocity component in user units
	 * as input to {@link #inverse(EnumMap)}.
	 * @return Velocity component in length distance
	 * as expected by {@link #inverse(EnumMap, double[])}.
	 */
	private double fromUserRobotSpeed(double userRobotSpeed) {
		return userRobotSpeed / robotSpeedFactor_;
	}

	/***
	 * Convert a robot rotation speed from user units to radians
	 * by dividing the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, double[])}.
	 * @param userRotationSpeed A robot rotational velocity in user units
	 * as input to {@link #inverse(EnumMap)}.
	 * @return Rotational velocity (radians)
	 * as expected by {@link #inverse(EnumMap, double[])}.
	 */
	private double fromUserRotationSpeed(double userRotationSpeed) {
		return userRotationSpeed / rotationSpeedFactor_;
	}

	/***
	 * Convert a wheel rotation speed from radians to user units
	 * by multiplying the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, double[])}.
	 * @param radiansPerSecond An individual wheel rotational velicity in radians
	 * as returned by {@link #inverse(EnumMap)}.
	 * @return Speed in user units as returned by {@link #inverse(EnumMap)}.
	 */
	private double toUserWheelSpeed(double radiansPerSecond) {
		return radiansPerSecond * wheelSpeedFactor_;
	}

	/***
	 * Convert a robot translation speed from distance
	 * (where distance is in the length/width/radius units passed to the constructor) to user units
	 * by multiplying the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, double[])}.
	 * @param unitsPerSecond A N/S or E/W robot translation velocity component in in distance
	 * as returned by {@link #forward(EnumMap)}.
	 * @return Speed in user units as returned by {@link #forward(EnumMap)}.
	 */
	private double toUserRobotSpeed(double unitsPerSecond) {
		return unitsPerSecond * robotSpeedFactor_;
	}

	/***
	 * Convert a robot rotation speed from radians to user units
	 * by multiplying the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, double[])}.
	 * @param radiansPerSecond A robot rotational velocity in radians
	 * as returned by {@link #forward(EnumMap)}.
	 * @return Speed in user units as returned by {@link #forward(EnumMap)}.
	 */
	private double toUserRotationSpeed(double radiansPerSecond) {
		return radiansPerSecond * rotationSpeedFactor_;
	}

	/**
	 * Compute robot-relative <b>[V]</b> ({north/south translation, east/west translation, rotation}) motion
	 * ncessary to reach location <b>[D]</b> ({north/south position, east/west position, heading})
	 * from initial location {0, 0, 0}, where translation and rotation speeds are constant or proportional
	 * throughout.
	 *
	 * @param userD <b>[D]</b>: total displacement {north/south translation, east/west translation, heading}, in user units.
	 * @return [3] robot-relative motion {north/south translation, east/west translation, rotation} in user units.
	 */
	public final double[] travel(double[] userD) {

		double[] d = {fromUserRobotSpeed(userD[0]), fromUserRobotSpeed(userD[1]), fromUserRotationSpeed(userD[2])};

		double headingChange = d[2];
		while(headingChange < -Math.PI) headingChange += 2 * Math.PI;
		while(headingChange > Math.PI) headingChange -= 2 * Math.PI;

		double displacementDistance = Math.sqrt(d[0] * d[0] + d[1] * d[1]);
		double displacementAngle = Math.atan2(d[1], d[0]);
		// portion of displacement angle due to curvature
		double curveAngle = Math.atan2(Math.sin(headingChange), 1 - Math.cos(headingChange));
		double travelDistance = headingChange == 0 ?
			displacementDistance :
			(displacementDistance * headingChange / (2 * Math.sin(headingChange / 2)));
		double travelAngle = displacementAngle - curveAngle;
		double travelN = travelDistance * Math.cos(travelAngle);
		double travelE = travelDistance * Math.sin(travelAngle);

		return new double[]{toUserRobotSpeed(travelN), toUserRobotSpeed(travelE), toUserRotationSpeed(headingChange)};
	}


	/***
	 * Compute robot-relative <b>[D]</b>  ({north/south position, east/west position, heading}) displacement
	 * for a motion <b>[V]</b> ({north/south translation, east/west translation, rotation})
	 * from initial location {0, 0, 0}, where translation and rotation speeds are constant or proportional
	 * throughout.
	 * @param translationVector <b>[V]</b>: {north/south translation, east/west translation, rotation}, in user units.
	 * @return [3] robot displacement {north/east translation, heading} in user units.
	 */
	public final double[] displacement(double[] translationVector) {

		double[] v = {
			fromUserRobotSpeed(translationVector[0]),
			fromUserRobotSpeed(translationVector[1]),
			fromUserRotationSpeed(translationVector[2])
		};

		double headingChange = v[2];
		while(headingChange < -Math.PI) headingChange += 2 * Math.PI;
		while(headingChange > Math.PI) headingChange -= 2 * Math.PI;

		double travelDistance = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
		double travelAngle = Math.atan2(v[1], v[0]);
		// portion of displacement angle due to curvature
		double curveAngle = Math.atan2(Math.sin(headingChange), 1 - Math.cos(headingChange));
		double displacementDistance = headingChange == 0 ?
			travelDistance :
			(travelDistance * (2 * Math.sin(headingChange / 2)) / headingChange);
		double displacementAngle = curveAngle + travelAngle;
		double displacementN = displacementDistance * Math.cos(displacementAngle);
		double displacementE = displacementDistance * Math.sin(displacementAngle);

		return new double[]{
			toUserRobotSpeed(displacementN),
			toUserRobotSpeed(displacementE),
			toUserRotationSpeed(headingChange)
		};
	}


	private static String toString(double[][] m) {
		StringBuffer sb = new StringBuffer();
		int rows = m.length;
		int cols = m[0].length;
		sb.append("\n{ ");
		for(int r = 0; r < rows; r++) {
			sb.append("\n{ ");
			for(int c = 0; c < cols; c++) {
				sb.append(roundd(m[r][c]));
				sb.append("\t");
			}
			sb.append("}");
		}
		sb.append("\n}");
		return sb.toString();
	}

	private static double[][] massage(double[][] m) {
		int rows = m.length;
		int cols = m[0].length;
		double[][] x = new double[rows][cols];
		for(int r = 0; r < rows; r++) {
			for(int c = 0; c < cols; c++) {
				x[r][c] = 1 / m[r][c] / 4;
			}
		}
		return x;
	}


	/**
	 * Matrix inversion (3x3 only).
	 *
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
			{
				m[1][1] * m[2][2] - m[1][2] * m[2][1],
				m[0][2] * m[2][1] - m[0][1] * m[2][2],
				m[0][1] * m[1][2] - m[0][2] * m[1][1]
			},
			{
				m[1][2] * m[2][0] - m[1][0] * m[2][2],
				m[0][0] * m[2][2] - m[0][2] * m[2][0],
				m[0][2] * m[1][0] - m[0][0] * m[1][2]
			},
			{
				m[1][0] * m[2][1] - m[1][1] * m[2][0],
				m[0][1] * m[2][0] - m[0][0] * m[2][1],
				m[0][0] * m[1][1] - m[0][1] * m[1][0]
			}
		};
		if(determinate != 0) {
			for(int i = 0; i < 3; i++) {
				for(int j = 0; j < 3; j++) {
					inverse[i][j] /= determinate;
				}
			}
		}
		return inverse;
	}


	/**
	 * Matrix transposition.
	 *
	 * @param m Matrix <b>[M]</b>.
	 * @return <b>[M]</b>'.
	 */
	private static double[][] transpose(double[][] m) {
		int rows = m[0].length;
		int cols = m.length;
		double[][] t = new double[rows][cols];
		for(int r = 0; r < rows; r++) {
			for(int c = 0; c < cols; c++) {
				t[r][c] = m[c][r];
			}
		}
		return t;
	}


	/**
	 * Matrix multiplication.
	 *
	 * @param a Matrix <b>[A]</b>.
	 * @param b Matrix <b>[B]</b>.
	 * @return <b>[A]</b>x<b>[B]</b>.
	 */
	public static double[][] multiply(double[][] a, double[][] b) {
		int n = b.length;
		int rows = a.length;
		int cols = b[0].length;
		double[][] p = new double[rows][cols];
		for(int r = 0; r < rows; r++) {
			for(int c = 0; c < cols; c++) {
				for(int i = 0; i < n; i++) {
					p[r][c] += a[r][i] * b[i][c];
				}
			}
		}
		return p;
	}


	public static double roundd(double d) {
		return Math.round(1000. * d) / 1000.;
	}
}
