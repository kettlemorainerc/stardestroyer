package org.usfirst.frc.team2077.math;

import org.junit.Test;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

import static org.junit.Assert.assertEquals;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.AssemblyPosition.NORTH_WEST;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.NORTH;

public class EnumMatrixTest {
	@Test
	public void transpose_as_expected() {
		EnumMatrix<AssemblyPosition, VelocityDirection> things = new EnumMatrix<>(AssemblyPosition.class, VelocityDirection.class);
		things.set(NORTH, NORTH_WEST, 123);

		EnumMatrix<VelocityDirection, AssemblyPosition> other_things = things.enumTranspose();

		assertEquals(other_things.getHeight(), things.getWidth());
		assertEquals(other_things.getWidth(), things.getHeight());
		assertEquals(other_things.get(NORTH_WEST, NORTH), things.get(NORTH, NORTH_WEST), 0);
	}
}
