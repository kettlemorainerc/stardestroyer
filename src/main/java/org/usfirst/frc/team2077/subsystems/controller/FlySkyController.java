package org.usfirst.frc.team2077.subsystems.controller;

import org.usfirst.frc.team2077.commands.RunGrabber;

import java.util.function.Function;

public class FlySkyController extends ControllerBinding {

	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
	 *
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public FlySkyController(int port) {
		super(port);
	}

	@Override
	public void bindDriver() {}

	@Override
	public double getAxis(Axis direction) {
		switch(direction) {
			case NORTH: return adjustInputSensitivity(getY(), .3, 1);
			case EAST: return adjustInputSensitivity(getX(), .3, 1);
			case ROTATION: return adjustInputSensitivity(getRawAxis(4), .05, 1);
			case GRABBER: return getZ();
		}
		return 0;
	}
}
