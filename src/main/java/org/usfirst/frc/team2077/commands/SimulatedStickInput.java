package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.*;

import java.time.Duration;

import static org.usfirst.frc.team2077.Robot.*;

public class SimulatedStickInput extends CommandBase {
	private final Duration simulateFor;
	private long startedAtMillis = 0;
	private Duration timeElapsed;
	// should be between 0-1
	private final double northThrottlePercent, eastThrottlePercent, rotationThrottlePercent;

	public SimulatedStickInput(double northThrottlePercent, double eastThrottlePercent, double rotationThrottlePercent) {
		this(Duration.ofMillis(500), northThrottlePercent, eastThrottlePercent, rotationThrottlePercent);
		addRequirements(robot_.position_, robot_.heading_);
	}

	public SimulatedStickInput(Duration simulateFor, double northThrottlePercent, double eastThrottlePercent, double rotationThrottlePercent) {
		System.out.println("Simulated stick input");
		this.simulateFor = simulateFor;
		this.northThrottlePercent = northThrottlePercent;
		this.eastThrottlePercent = eastThrottlePercent;
		this.rotationThrottlePercent = rotationThrottlePercent;
	}

	public void initialize() {
	}

	@Override
	public void execute() {
		if(startedAtMillis == 0) startedAtMillis = System.currentTimeMillis();
		timeElapsed = Duration.ofMillis(System.currentTimeMillis() - startedAtMillis);
		robot_.chassis_.setGLimits(PrimaryStickDrive3Axis.ACCELERATION_G_LIMIT, PrimaryStickDrive3Axis.DECELERATION_G_LIMIT);

		System.out.printf(
			"[N%% | E%% | R%%][%s | %s | %s][Using R%%: %s]%n",
			northThrottlePercent,
			eastThrottlePercent,
			rotationThrottlePercent,
			CommandScheduler.getInstance().requiring(robot_.heading_) == null
		);

		if(CommandScheduler.getInstance().requiring(robot_.heading_) != null) { // we don't control heading
			robot_.chassis_.setVelocity01(northThrottlePercent, eastThrottlePercent);
		} else { // we control heading
			robot_.chassis_.setVelocity01(
				northThrottlePercent,
				eastThrottlePercent,
				rotationThrottlePercent
			);
		}
	}

	@Override
	public void end(boolean interrupted) {
		robot_.chassis_.halt();
	}

	@Override
	public boolean isFinished() {
		System.out.println("elapsed: " + timeElapsed);
		System.out.println("for: " + simulateFor);
		return timeElapsed.compareTo(simulateFor) < 0;
	}
}
