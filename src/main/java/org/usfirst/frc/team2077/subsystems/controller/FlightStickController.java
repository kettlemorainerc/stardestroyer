package org.usfirst.frc.team2077.subsystems.controller;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class FlightStickController extends ControllerBinding {
	private JoystickButton grabber = null;
	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
	 *
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public FlightStickController(int port) {
		super(port);
	}

	@Override
	public void bindDriver() {
		super.bindDriver();
	}

	@Override
	public void bindTechnical() {
		super.bindTechnical();
	}

	@Override
	public double getAxis(Axis direction) {
		switch(direction) {
			case NORTH: return adjustInputSensitivity(getY(), .2, 2.5);
			case EAST: return adjustInputSensitivity(getX(), .2, 2.5);
			case ROTATION: return adjustInputSensitivity(getRawAxis(2), .2, 2.5);
			case GRABBER: return grabber != null && grabber.get() ? 1 : 0;
		}
		return super.getAxis(direction);
	}
}
