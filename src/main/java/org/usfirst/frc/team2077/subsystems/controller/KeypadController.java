package org.usfirst.frc.team2077.subsystems.controller;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.commands.*;

import java.util.List;
import java.util.function.*;

public class KeypadController extends ControllerBinding {
	private final JoystickButtonAxis north, east, rotation, grabber;

	// TODO: Figure out if we want different buttons depending on whether we're driver or technical (or if that's even relevant)
	public enum SupportedAxis {
		NORTH(2, 6),
		EAST(5, 7),
		// TODO: Figure out if we even care about rotation on a keypad
		ROTATION(null, null),
		GRABBER(9, null);

		private final Integer positive, negative;
		SupportedAxis(Integer positive, Integer negative) {
			this.positive = positive;
			this.negative = negative;
		}
	}
	/**
	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
	 *
	 * @param port The port on the Driver Station that the joystick is plugged into.
	 */
	public KeypadController(int port, SupportedAxis... axes) {
		super(port);

		List<SupportedAxis> allAxes = List.of(axes);

		north = axisWithButtons(allAxes, SupportedAxis.NORTH);
		east = axisWithButtons(allAxes, SupportedAxis.EAST);
		rotation = axisWithButtons(allAxes, SupportedAxis.ROTATION);
		grabber = axisWithButtons(allAxes, SupportedAxis.GRABBER);
	}

	private JoystickButtonAxis axisWithButtons(List<SupportedAxis> axes, SupportedAxis axis) {
		return axes.contains(axis) ? new JoystickButtonAxis(this, axis.positive, axis.negative) : JoystickButtonAxis.none;
	}

	@Override
	public void bindTechnical() {
		bindCommandToButton(1, new TurnOffLauncher(), JoystickButton::whenPressed);
		bindCommandToButton(3, new Launch(), JoystickButton::whileHeld);
		bindCommandToButton(4, new LoadLauncher(), JoystickButton::whileHeld);
		bindCommandToButton(8, new LoadLauncherBack(), JoystickButton::whenPressed);
		bindCommandToButton(10, new SteerToCrosshairs(), JoystickButton::whileHeld);
		bindCommandToButton(11, new CenterBallOnVision(), JoystickButton::whileHeld);
		bindCommandToButton(12, new LauncherScrewTest(false), JoystickButton::whileHeld);
		bindCommandToButton(16, new LauncherScrewTest(true), JoystickButton::whileHeld);
	}

	@Override
	public double getAxis(Axis direction) {
		switch(direction) {
			case NORTH: return north.getAxis();
			case EAST: return east.getAxis();
			case ROTATION: return rotation.getAxis();
			case GRABBER: return grabber.getAxis();
		}
		return super.getAxis(direction);
	}

	protected static class JoystickButtonAxis {
		private static final JoystickButtonAxis none = new JoystickButtonAxis(null, null, null);
		private final BooleanSupplier positive, negative;

		protected JoystickButtonAxis(KeypadController parent, Integer positive, Integer negative) {
			this.positive = positive == null ? () -> false : new JoystickButton(parent, positive)::get;
			this.negative = negative == null ? () -> false : new JoystickButton(parent, negative)::get;
		}

		protected double getAxis() {
			return positive.getAsBoolean() ? 1 : negative.getAsBoolean() ? -1 : 0;
		}
	}
}
