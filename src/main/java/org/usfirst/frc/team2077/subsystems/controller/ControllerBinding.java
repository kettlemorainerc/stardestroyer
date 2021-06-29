package org.usfirst.frc.team2077.subsystems.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.*;

public abstract class ControllerBinding extends Joystick {
	public enum Axis {NORTH, EAST, ROTATION, GRABBER}

	@SafeVarargs
	protected final void bindCommandToButton(int buttonNumber, Command c, BiConsumer<JoystickButton, Command>... forActions) {
		JoystickButton button = new JoystickButton(this, buttonNumber);

		for(BiConsumer<JoystickButton, Command> action : forActions) action.accept(button, c);
	}

	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
	 *
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public ControllerBinding(int port) {
		super(port);
	}

	public void bindDriver() {
		System.out.println("Bind Driver Unimplemented for " + getClass().getName());
	}
	public void bindTechnical() {
		System.out.println("Bind Technical Unimplemented for " + getClass().getName());
	}
	public double getAxis(Axis direction) {return 0d;}

	public void cancelDriver() {}
	public void cancelTechnical() {}

	/**
	 * Condition control axis input to improve driveability.
	 * Each axis has a center dead band in which the output for that axis is always zero.
	 * Outside the dead band the output increases exponentially from zero to 1 or -1.
	 */
	public static double adjustInputSensitivity(double input, double deadBand, double exponent) {
		return Math.pow(Math.max(0, Math.abs(input) - deadBand) / (1 - deadBand), exponent) * Math.signum(input);
	}
}
