package org.usfirst.frc.team2077.drivetrain;

import java.util.*;

public class TestClock {
	private static double seconds = 0;

	public static double getAndIncrementSeconds() {
		seconds += .2;
		return seconds;
	}

	public static double getSeconds() {
		return seconds;
	}

	public static void reset() {
		seconds = 0;
	}
}
