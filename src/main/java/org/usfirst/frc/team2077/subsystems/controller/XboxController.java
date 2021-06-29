package org.usfirst.frc.team2077.subsystems.controller;

public final class XboxController extends ControllerBinding {
	public enum Button {
		A(1),
		B(2),
		X(3),
		Y(4),
		LEFT_BUMPER(5),
		RIGHT_BUMPER(6),
		SELECT(7),
		START(8),
		LEFT_JOYSTICK(9),
		RIGHT_JOYSTICK(10);

		private int port;

		Button(int port) {this.port = port;}
	}

	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
	 *
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	private XboxController(int port) {
		super(port);
	}

	@Override
	public double getAxis(Axis direction) {
		switch(direction) { // TODO: calibrate xbox dead zones
			case NORTH: return adjustInputSensitivity(getX(),.15, 1);
			case EAST: return adjustInputSensitivity(getY(), .15, 1);
			case ROTATION: return adjustInputSensitivity(getRawAxis(4), .15, 1);
			case GRABBER: return adjustInputSensitivity(getRawAxis(5), .15, 1);
		}

		throw new IllegalArgumentException("Axis direction " + direction + " not supported");
	}
}
