package org.usfirst.frc.team2077.math;

import java.util.*;
import java.util.function.Function;
import java.util.stream.*;

import static java.util.Set.*;

public class Matrix {
	double[][] matrix;

	public Matrix(int height, int width) {
		/* 3 x 1 matrix
		double[1][3] = {
		 {0},
		 {0},
		 {0}
		};
		 */
		this(new double[height][width]);
	}

	private Matrix(Matrix toTranspose) {
		double[][] original = toTranspose.matrix;
		double[][] transposedMatrix = new double[original.length][original[0].length];

		for(int x = 0; x < original.length; x++) {
			for(int y = 0; y < transposedMatrix.length; y++) {
				transposedMatrix[y][x] = original[x][y];
			}
		}

		matrix = transposedMatrix;
	}

	public Matrix(Enum<? extends Enum> height, Enum<? extends Enum> width) {
		this(
			height.getClass().getEnumConstants().length,
			width.getClass().getEnumConstants().length
		);
	}

	public Matrix(double[][] matrix) {
		this.matrix = matrix;
	}

	public Matrix transpose() {
		return new Matrix(this);
	}

	/**
	 * This directly correlates to valid "X" arguments made to
	 * {@link #get(int, int)}/{@link #set(int, int, double)}/{@link #calculate(int, int, Function)}
	 *
	 * @throws IndexOutOfBoundsException if the height of the matrix is 0
	 * @return the "width" of the matrix
	 */
	public int getWidth() {
		return matrix[0].length;
	}

	/**
	 * This directly correlates to valid "Y" arguments made to
	 * {@link #get(int, int)}/{@link #set(int, int, double)}/{@link #calculate(int, int, Function)}
	 * @return the "height" of this matrix
	 */
	public int getHeight() {
		return matrix.length;
	}

	public double get(int x, int y) {
		return matrix[y][x];
	}

	public void set(int x, int y, double value) {
		matrix[y][x] = value;
	}

	public double get(Enum<? extends Enum> x, Enum<? extends Enum> y) {
		return get(x.ordinal(), y.ordinal());
	}

	public void set(Enum<? extends Enum> x, Enum<? extends Enum> y, double value) {
		set(x.ordinal(), y.ordinal(), value);
	}

	public void calculate(int x, int y, Function<Double, Double> calculate) {
		double value;
		value = get(x, y);

		set(x, y, calculate.apply(value));
	}

	/**
	 * this assumes the passed matrix is a compatible matrix and performs no checks on dimensions
	 * @param by the matrix to multiply by
	 * @return a this.getHeight X by.getWidth matrix
	 */
	public Matrix multiply(Matrix by) {
		Matrix result = new Matrix(getHeight(), by.getWidth());

		for (int x = 0; x < result.getWidth(); x++) {
			for(int y = 0; y < result.getHeight(); y++) {
				for(int col = 0; col < getWidth(); col++) {
					double toAdd = get(col, y) * by.get(x, col);
					result.calculate(x, y, cur -> cur + toAdd);
				}
			}
		}

		return result;
	}

	public Matrix multiply(double by) {
		Matrix result = new Matrix(getHeight(), getWidth());

		for(int x = 0; x < result.getWidth(); x++) {
			for(int y = 0; y < result.getHeight(); y++) {
				result.set(x, y, get(x, y) * by);
			}
		}

		return result;
	}

	public double[][] getMatrix() {
		double[][] result = new double[getWidth()][getHeight()];

		for(int x = 0; x < getWidth(); x++)
			for(int y = 0; y < getHeight(); y++) result[x][y] = get(x, y);

		return result;
	}

	@Override
	public boolean equals(Object o) {
		if(this == o) return true;
		if(o == null || getClass() != o.getClass()) return false;
		Matrix that = (Matrix) o;
		return Arrays.deepEquals(this.getMatrix(), that.getMatrix());
	}

	@Override
	public int hashCode() {
		return Arrays.deepHashCode(matrix);
	}

	@Override
	public String toString() {
		Object[] formatArgs = new Object[getHeight() * getWidth()];
		int longest = "0.00".length();
		StringBuilder output = new StringBuilder();
		String formatToLengthFormat = "%%%1$d.2f"; // %1$d will be the initial conversion for lengths

		for(int y = 0; y < getHeight(); y++) {
			output.append("| ");
			for(int x = 0; x < getWidth(); x++) {
				if(x > 0) output.append(", ");
				output.append(formatToLengthFormat);

				// the current longest vs the current value to 2 decimal places
				longest = Math.max(longest, String.format("%.2f", get(x, y)).length());
				formatArgs[(getWidth() * y) + x] = get(x, y);
			}
			output.append(" |%%n");
		}
		String emDash = "\u2014";
		String padding = emDash + emDash;
		StringBuilder line = new StringBuilder(padding);
		for(int i = 0; i < getWidth(); i++) {
			line.append(emDash.repeat(longest))
			    .append(padding); // either the ", " or the " |"
		}
		String format = String.format(output.toString(), longest);
		return line.toString() + '\n' + String.format(format, formatArgs) + line;
	}

	public Matrix inverse3x3() {
		double[][] m = matrix;
		double determinate
			= m[0][0] * m[1][1] * m[2][2]
			  + m[0][1] * m[1][2] * m[2][0]
			  + m[0][2] * m[1][0] * m[2][1]
			  - m[2][0] * m[1][1] * m[0][2]
			  - m[2][1] * m[1][2] * m[0][0]
			  - m[2][2] * m[1][0] * m[0][1];
		double[][] inverse = {
			{
				m[1][1] * m[2][2] - m[1][2] * m[2][1],
				m[0][2] * m[2][1] - m[0][1] * m[2][2],
				m[0][1] * m[1][2] - m[0][2] * m[1][1]
			},
			{
				m[1][2] * m[2][0] - m[1][0] * m[2][2],
				m[0][0] * m[2][2] - m[0][2] * m[2][0],
				m[0][2] * m[1][0] - m[0][0] * m[1][2]
			},
			{
				m[1][0] * m[2][1] - m[1][1] * m[2][0],
				m[0][1] * m[2][0] - m[0][0] * m[2][1],
				m[0][0] * m[1][1] - m[0][1] * m[1][0]
			}
		};
		if(determinate != 0) {
			for(int i = 0; i < 3; i++) {
				for(int j = 0; j < 3; j++) {
					inverse[i][j] /= determinate;
				}
			}
		}
		return new Matrix(inverse);
	}
}
