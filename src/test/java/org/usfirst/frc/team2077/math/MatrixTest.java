package org.usfirst.frc.team2077.math;

import org.junit.*;

public class MatrixTest {

	@Test
	public void product_has_inner_dimensions() {
		Matrix fourByThree = new Matrix(4, 3);
		Matrix threeByFour = new Matrix(3, 4);

		Matrix threeByThree = fourByThree.multiply(threeByFour);
		Matrix fourByFour = threeByFour.multiply(fourByThree);

		Assert.assertEquals(3, threeByThree.getWidth());
		Assert.assertEquals(3, threeByThree.getHeight());

		Assert.assertEquals(4, fourByFour.getWidth());
		Assert.assertEquals(4, fourByFour.getHeight());
	}

	@Test
	public void matrices_are_equal_when_their_contained_arrays_and_transposition_are() {
		Matrix a = new Matrix(new double[][] {
			{1, 2},
			{3, 4}
		});

		Matrix b = new Matrix(new double[][] {
			{1, 2},
			{3, 4}
		});

		Assert.assertEquals(a, b);
		b.transpose();
		Assert.assertNotEquals(a, b);
		b.transpose();
		Assert.assertEquals(a, b);
	}

	@Test
	public void produce_matches_expected_algorithm() {
		Matrix a = new Matrix(new double[][] { // 2 x 3
			{2, 4},
			{6, 8},
			{10, 12}
		});

		Matrix b = new Matrix(new double[][] { // 3 x 2
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

		System.out.println(aByBResult);
		Assert.assertEquals(aByBResult, a.multiply(b));
		Assert.assertEquals(bByAResult, b.multiply(a));
	}
}
