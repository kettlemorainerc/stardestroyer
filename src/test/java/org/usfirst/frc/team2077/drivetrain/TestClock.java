package org.usfirst.frc.team2077.drivetrain;

import java.util.*;

public class TestClock {
	private static double seconds = 0;

	private static String[] sub(StackTraceElement[] trace, int from, int to) {
		int length = Math.min(trace.length, to - from);
		String[] stack = new String[length];

		for(int i = 0; i < length; i++) {
			stack[i] = trace[from + i].toString();
		}

		return stack;
	}

	public static double getAndIncrementSeconds() {
		seconds += .2;
		System.out.printf("\n\t%s\n%n",
			String.join("\n\t", sub(new Exception().getStackTrace(), 1, 11))
		);
		return seconds;
	}

	public static double getSeconds() {
		return seconds;
	}

	public static void reset() {
		seconds = 0;
	}
}
