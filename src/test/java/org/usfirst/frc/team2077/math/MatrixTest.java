package org.usfirst.frc.team2077.math;

import org.junit.*;

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
	public void matrices_are_equal_when_their_internal_arrays_are() {
		Matrix a = new Matrix(new double[][] {
			{1, 2},
			{3, 4}
		});

		Matrix b = new Matrix(new double[][] {
			{1, 2},
			{3, 4}
		});

		assertEquals(a, b);
		Matrix bTranspose = b.transpose();
		Assert.assertNotEquals(a, bTranspose);
		Matrix aTranspose = a.transpose();
		assertEquals(aTranspose, bTranspose);
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

		assertEquals(aByBResult, a.multiply(b));
		assertEquals(bByAResult, b.multiply(a));
		assertEquals(aBy2Result, a.multiply(2d));
	}

	@Test
	public void determinate_is_as_expected() {
		Matrix twoByTwo = new Matrix(new double[][]{
			{2, 1},
			{8, 5}
		});

		Matrix threeByThree = new Matrix(new double[][]{
			{3, 2, 5},
			{5, 1, 7},
			{9, 5, 2}
		});

		assertEquals(2, Matrix.determinate(twoByTwo), .0003);
		assertEquals(87, Matrix.determinate(threeByThree), .0003);
		double math = 0d, generic = 0d;

		long start = System.nanoTime();
		for(int i = 0; i < 100; i++) math = determinate(threeByThree);
		long end = System.nanoTime();
		System.out.println("Math took: " + (end - start));

		start = System.nanoTime();
		for(int i = 0; i < 100; i++) generic = Matrix.determinate(threeByThree);
		end = System.nanoTime();

		System.out.println("Generic took: " + (end - start));
		assertEquals(generic, math, .0003);
	}

	private static double determinate(Matrix of) {
		double[][] m = of.matrix;
		return m[0][0] * m[1][1] * m[2][2]
			  + m[0][1] * m[1][2] * m[2][0]
			  + m[0][2] * m[1][0] * m[2][1]
			  - m[2][0] * m[1][1] * m[0][2]
			  - m[2][1] * m[1][2] * m[0][0]
			  - m[2][2] * m[1][0] * m[0][1];
	}
}
