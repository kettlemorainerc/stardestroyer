package org.usfirst.frc.team2077.math;

import org.junit.Test;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

import static org.junit.Assert.assertEquals;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.WheelPosition.NORTH_WEST;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction.NORTH;

public class EnumMatrixTest {
	@Test
	public void transpose_as_expected() {
		EnumMatrix<Direction, WheelPosition> things = new EnumMatrix<>(Direction.class, WheelPosition.class);
		things.set(NORTH, NORTH_WEST, 123);

		EnumMatrix<WheelPosition, Direction> other_things = things.enumTranspose();

		assertEquals(other_things.getHeight(), things.getWidth());
		assertEquals(other_things.getWidth(), things.getHeight());
		assertEquals(other_things.get(NORTH_WEST, NORTH), things.get(NORTH, NORTH_WEST), 0);
	}
}
