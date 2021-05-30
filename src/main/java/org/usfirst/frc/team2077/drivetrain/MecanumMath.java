package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.math.EnumMatrix;

import java.util.EnumMap;

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.WheelPosition.*;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;

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
 *  <dt>{@link #inverse(EnumMap, Point)}</dt>
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
 *  <dt>{@linkplain Point#inverseMatrixForBotSize(double, double)}</dt>
 *  <dd><p style="margin-left: 40px">Convenience methods for initializing the inverse kinematic matrix <b>[R]</b>.</p></dd>
 *  <dt>{@link #createForwardMatrix(EnumMatrix)}</dt>
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
	public enum WheelPosition {
		NORTH_EAST,
		SOUTH_EAST,
		SOUTH_WEST,
		NORTH_WEST,
	}

	public enum VelocityDirection {
		NORTH,
		EAST, ROTATION,
	}

	public static class Point {
		public final double north, east;

		/**
		 * (0, 0) is assumed to be the <strong>center</strong> of the bot in question
		 * @param north coordinate
		 * @param east coordinate
		 */
		public Point(double north, double east) {
			this.north = north;
			this.east = east;
		}
		
		public EnumMatrix<VelocityDirection, WheelPosition> inverseMatrixForBotSize(double length, double width) {
			EnumMatrix<WheelPosition, VelocityDirection> wheelCoords = new EnumMatrix<>(WheelPosition.class, VelocityDirection.class);
			wheelCoords.set(NORTH_EAST, NORTH, length / 2 - north);
			wheelCoords.set(NORTH_EAST, EAST, width / 2 - east);
			wheelCoords.set(NORTH_WEST, NORTH, length / 2 - north);
			wheelCoords.set(NORTH_WEST, EAST, -width / 2 - east);

			wheelCoords.set(SOUTH_EAST, NORTH, -length / 2 - north);
			wheelCoords.set(SOUTH_EAST, EAST, width / 2 - east);
			wheelCoords.set(SOUTH_WEST, NORTH, -length / 2 - north);
			wheelCoords.set(SOUTH_WEST, EAST, -width / 2 - east);

			EnumMatrix<VelocityDirection, WheelPosition> inverseMatrix = new EnumMatrix<>(
				VelocityDirection.class,
				WheelPosition.class
			);
			inverseMatrix.set(NORTH, NORTH_EAST, 1d);
			inverseMatrix.set(EAST, NORTH_EAST, -1d);
			inverseMatrix.set(
				ROTATION,
				NORTH_EAST,
				-wheelCoords.get(NORTH_EAST, NORTH) - wheelCoords.get(NORTH_EAST, EAST)
			);
			inverseMatrix.set(NORTH, NORTH_WEST, 1d);
			inverseMatrix.set(EAST, NORTH_WEST, 1d);
			inverseMatrix.set(
				ROTATION,
				NORTH_WEST,
				wheelCoords.get(NORTH_WEST, NORTH) - wheelCoords.get(NORTH_WEST, EAST)
			);

			inverseMatrix.set(NORTH, SOUTH_EAST, 1d);
			inverseMatrix.set(EAST, SOUTH_EAST, 1d);
			inverseMatrix.set(
				ROTATION,
				SOUTH_EAST,
				wheelCoords.get(SOUTH_EAST, NORTH) - wheelCoords.get(SOUTH_EAST, EAST)
			);
			inverseMatrix.set(NORTH, SOUTH_WEST, 1d);
			inverseMatrix.set(EAST, SOUTH_WEST, -1d);
			inverseMatrix.set(
				ROTATION,
				SOUTH_WEST,
				-wheelCoords.get(SOUTH_WEST, NORTH) - wheelCoords.get(SOUTH_WEST, EAST)
			);

			return inverseMatrix;
		}
	}

	/**
	 * Construct the forward matrix <b>[F]</b> for a rectangular robot.
	 *
	 * Performs <b>(([R]<sup>T</sup>[R])[R]<sup>T</sup>)</b> to calculate <b>[F]</b>
	 *
	 * @param inverseKinematicMatrix <b>[R]</b> - Inverse kinematic matrix 4x3.
	 * @return <b>[F]</b> - 3x4 forward kinematic matrix for use by {@link #forward}.
	 */
	public static EnumMatrix<WheelPosition, VelocityDirection> createForwardMatrix(EnumMatrix<VelocityDirection, WheelPosition> inverseKinematicMatrix) {
		EnumMatrix<WheelPosition, VelocityDirection> transposedInverse = inverseKinematicMatrix.enumTranspose();
		return transposedInverse.enumMultiply(inverseKinematicMatrix)
								.enumInvert3x3()
								.enumMultiply(transposedInverse);
	}

	private final EnumMatrix<VelocityDirection, WheelPosition> reverseMatrix_;
	private final EnumMatrix<WheelPosition, VelocityDirection> forwardMatrix_;
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
		this(length, width, wheelRadius, wheelSpeedFactor, robotSpeedFactor, rotationSpeedFactor, new Point(0, 0));
	}

	/**
	 * Initialize a robot-specific context for inverse and forward kinematics solutions,
	 * with the center of rotation at a specified point.
	 * This constructor is equivalent to
	 * <code>MecanumMath(length,width,wheelRadius,1,1,1,rotationCenter)</code>.
	 *  @param length         Distance between wheel contact points in the N/S direction, in any distance unit.
	 * @param width          Distance between wheel contact points in the E/W direction, in the same unit.
	 * @param wheelRadius    Radius of each wheel, in the same unit.
	 * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
	 */
	public MecanumMath(double length, double width, double wheelRadius, Point rotationCenter) {
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
	public MecanumMath(double length, double width, double wheelRadius, double wheelSpeedFactor, double robotSpeedFactor, double rotationSpeedFactor, Point rotationCenter) {
		length_ = length;
		width_ = width;
		reverseMatrix_ = rotationCenter.inverseMatrixForBotSize(length, width);
		//forwardMatrix_ = createForwardMatrix(length, width);
		forwardMatrix_ = createForwardMatrix(reverseMatrix_);
		wheelRadius_ = wheelRadius;
		wheelSpeedFactor_ = wheelSpeedFactor;
		robotSpeedFactor_ = robotSpeedFactor;
		rotationSpeedFactor_ = rotationSpeedFactor;
	}

	/***
	 * Solve the inverse kinematic equation <b>[&Omega;] = (1/r)[R][V]</b>.
	 * This method wraps the static {@link #inverse(EnumMap, Point)} method, passing the internal
	 * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
	 * values from and to user units if conversion factors were specified.
	 * @param translationMatrix <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
	 * @return The wheel speed vector <b>[&Omega;]</b>: [NE, SE, SW, NW] in user units.
	 */
	public final EnumMap<WheelPosition, Double> inverse(EnumMap<VelocityDirection, Double> translationMatrix) {
		return inverse(translationMatrix, new Point(0, 0));
	}

	/***
	 * Solve the inverse kinematic equation <b>[&Omega;] = (1/r)[R][V]</b> for rotation about an arbitrary point.
	 * <b>[R]</b> matrix and wheel radius initialized in the constructor, and converting input and output
	 * values from and to user units if conversion factors were specified.
	 * @param translationMatrix <b>[V]</b>: [north/south translation, east/west translation, rotation], in user units.
	 * @param rotationCenter {N,E} coordinates of the center of rotation relative to the robot's geometric center.
	 * @return The wheel speed vector <b>[&Omega;]</b>: [NE, SE, SW, NW] in user units.
	 */
	private EnumMap<WheelPosition, Double> inverse(EnumMap<VelocityDirection, Double> translationMatrix, Point rotationCenter) {
		EnumMatrix<VelocityDirection, WheelPosition> inverseKineticMatrix =
			rotationCenter.north == 0 && rotationCenter.east == 0 ?
				reverseMatrix_ :
				rotationCenter.inverseMatrixForBotSize(length_, width_);

		EnumMap<WheelPosition, Double> wheelSpeedVector = new EnumMap<>(WheelPosition.class);
		for(WheelPosition position : WheelPosition.values()) {
			wheelSpeedVector.put(position, 0d);

			for(VelocityDirection direction : VelocityDirection.values()) {
				wheelSpeedVector.compute(
					position,
					(key, val) -> {
						double adjustedValue = direction == ROTATION ?
						                       fromUserRotationSpeed(translationMatrix.get(direction)) :
						                       fromUserRobotSpeed(translationMatrix.get(direction));
						return val + (adjustedValue * inverseKineticMatrix.get(direction, position)) / wheelRadius_;
					}
				);
			}
		}

		for(WheelPosition position : WheelPosition.values()){
			wheelSpeedVector.compute(position, (k, v) -> toUserWheelSpeed(v));
		}

		return wheelSpeedVector;
	}

	/***
	 * Solve the forward kinematic equation <b>[V] = [F][&Omega;](r)</b>.
	 * @param wheelSpeeds A wheel speed vector <b>[&Omega;]</b>: [NE, SE, SW, NW] in user units.
	 * @return [3] robot motion velocities [north/south translation, east/west translation, rotation] in user units.
	 */
	public final EnumMap<VelocityDirection, Double> forward(EnumMap<WheelPosition, Double> wheelSpeeds) {
		EnumMap<VelocityDirection, Double> translation = new EnumMap<>(VelocityDirection.class);

		for(VelocityDirection direction: VelocityDirection.values()) {
			translation.put(direction, 0d);
			for(WheelPosition position : WheelPosition.values()) {
				translation.compute(
					direction,
					(k, val) -> (
						val + (fromUserWheelSpeed(wheelSpeeds.get(position)) *
						       forwardMatrix_.get(position, direction) *
						       wheelRadius_)
					)
				);
			}
		}

		for(VelocityDirection direction : VelocityDirection.values()) {
			translation.compute(
				direction,
				(k, v) -> direction == ROTATION ? toUserRotationSpeed(v) : toUserRobotSpeed(v)
			);
		}

		return translation;
	}

	/***
	 * Convert a wheel rotation speed from user units to radians
	 * by dividing the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, Point)}.
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
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, Point)}.
	 * @param userRobotSpeed A N/S or E/W robot translation velocity component in user units
	 * as input to {@link #inverse(EnumMap)}.
	 * @return Velocity component in length distance
	 * as expected by {@link #inverse(EnumMap, Point)}.
	 */
	private double fromUserRobotSpeed(double userRobotSpeed) {
		return userRobotSpeed / robotSpeedFactor_;
	}

	/***
	 * Convert a wheel rotation speed from radians to user units
	 * by multiplying the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, Point)}.
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
	 * constructor {@link #MecanumMath(double, double, double, double, double, double)}.
	 * @param unitsPerSecond A N/S or E/W robot translation velocity component in in distance
	 * as returned by {@link #forward(EnumMap)}.
	 * @return Speed in user units as returned by {@link #forward(EnumMap)}.
	 */
	private double toUserRobotSpeed(double unitsPerSecond) {
		return unitsPerSecond * robotSpeedFactor_;
	}

	/***
	 * Convert a robot rotation speed from user units to radians
	 * by dividing the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double, Point)}
	 * @param userRotationSpeed A robot rotational velocity in user units
	 * as input to {@link #inverse(EnumMap)}.
	 * @return Rotational velocity (radians)
	 * as expected by {@link #inverse(EnumMap)}.
	 */
	private double fromUserRotationSpeed(double userRotationSpeed) {
		return userRotationSpeed / rotationSpeedFactor_;
	}

	/***
	 * Convert a robot rotation speed from radians to user units
	 * by multiplying the input value by the conversion factor passed to the
	 * constructor {@link #MecanumMath(double, double, double, double, double, double)}.
	 * @param radiansPerSecond A robot rotational velocity in radians
	 * as returned by {@link #forward(EnumMap)}.
	 * @return Speed in user units as returned by {@link #forward(EnumMap)}.
	 */
	private double toUserRotationSpeed(double radiansPerSecond) {
		return radiansPerSecond * rotationSpeedFactor_;
	}
}
