package org.usfirst.frc.team2077.math;

import org.junit.*;

import java.util.Arrays;

import static org.junit.Assert.*;

public class MatrixTest {

	@Test
	public void product_matches_proper_dimensions() {
		Matrix fourByThree = new Matrix(4, 3);
		Matrix threeByFour = new Matrix(3, 4);

		Matrix fourByFour = fourByThree.multiply(threeByFour);
		Matrix threeByThree = threeByFour.multiply(fourByThree);

		assertEquals(3, threeByThree.getWidth());
		assertEquals(3, threeByThree.getHeight());

		assertEquals(4, fourByFour.getWidth());
		assertEquals(4, fourByFour.getHeight());
	}

	@Test
	public void transpose_flips_x_and_y() {
		Matrix a = new Matrix(new double[][] {
			{1, 2, 3},
			{4, 5, 6}
		});

		Matrix b = new Matrix(new double[][] {
			{1, 4},
			{2, 5},
			{3, 6}
		});

		assertNotEquals(a, b);
		assertEquals(a, b.transpose());
		Matrix aTranspose = a.transpose();
		assertEquals(aTranspose, b);
	}

	@Test
	public void matrices_are_equal_when_their_internal_arrays_are() {
		Matrix a = new Matrix(new double[][] {
			{1, 2, 3},
			{4, 5, 6}
		});

		Matrix b = new Matrix(new double[][] {
			{1, 4},
			{2, 5},
			{3, 6}
		});

		Matrix c = new Matrix(new double[][] {
			{1, 2, 3},
			{4, 5, 6}
		});

		assertNotEquals(a, b);
		assertEquals(a, c);
		assertEquals(b, new Matrix(b.matrix));
	}

	@Test
	public void get_matrix_provides_equivalent_matrix() {
		double[][] example = {
			{1, 2, 3},
			{4, 5, 6}
		};
		Matrix matrix = new Matrix(example);

		assertTrue(Arrays.deepEquals(example, matrix.getMatrix()));
	}

	@Test
	public void matrix_properly_calculates_product_values() {
		Matrix a = new Matrix(new double[][] { // 3 x 2
			{2, 4},
			{6, 8},
			{10, 12}
		});

		Matrix b = new Matrix(new double[][] { // 2 x 3
			{1, 3, 5},
			{7, 9, 11}
		});

		Matrix aByBResult = new Matrix(new double[][] {
			{(2 * 1) + (4 * 7), (2 * 3) + (4 * 9), (2 * 5) + (4 * 11)},
			{(6 * 1) + (8 * 7), (6 * 3) + (8 * 9), (6 * 5) + (8 * 11)},
			{(10 * 1) + (12 * 7), (10 * 3) + (12 * 9), (10 * 5) + (12 * 11)}
		});

		Matrix bByAResult = new Matrix(new double[][] {
			{(1 * 2) + (3 * 6) + (5 * 10), (1 * 4) + (3 * 8) + (5 * 12)},
			{(7 * 2) + (9 * 6) + (11 * 10), (7 * 4) + (9 * 8) + (11 * 12)}
		});

		Matrix aBy2Result = new Matrix(new double[][] {
			{4, 8},
			{12, 16},
			{20, 24}
		});

		System.out.println(aBy2Result);
		assertEquals(aByBResult, a.multiply(b));
		assertEquals(bByAResult, b.multiply(a));
		assertEquals(aBy2Result, a.multiply(2d));
	}
}
