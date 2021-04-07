package org.usfirst.frc.team2077.math;

import java.util.*;
import java.util.function.Function;

public class Matrix {
	double[][] matrix;
	boolean transposed = false;

	public Matrix(int height, int width) {
		/* 3 x 1 matrix
		double[1][3] = {
		 {0},
		 {0},
		 {0}
		};
		 */
		this(new double[width][height]);
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

	public void transpose() {
		transposed = !transposed;
	}

	/**
	 * This directly correlates to valid "X" arguments made to
	 * {@link #get(int, int)}/{@link #set(int, int, double)}/{@link #calculate(int, int, Function)}
	 * @return the "width" of the matrix
	 */
	public int getWidth() {
		if (transposed) return matrix.length;
		return matrix[0].length;
	}

	/**
	 * This directly correlates to valid "Y" arguments made to
	 * {@link #get(int, int)}/{@link #set(int, int, double)}/{@link #calculate(int, int, Function)}
	 * @return the "height" of this matrix
	 */
	public int getHeight() {
		if(transposed) return matrix[0].length;
		return matrix.length;
	}

	public double get(int x, int y) {
		if(transposed) return matrix[y][x];
		return matrix[x][y];
	}

	public void set(int x, int y, double value) {
		if(transposed) matrix[y][x] = value;
		else matrix[x][y] = value;
	}

	public double get(Enum<? extends Enum> x, Enum<? extends Enum> y) {
		if(transposed) return matrix[y.ordinal()][x.ordinal()];

		return matrix[x.ordinal()][y.ordinal()];
	}

	public void set(Enum<? extends Enum> x, Enum<? extends Enum> y, double value) {
		if(transposed) matrix[y.ordinal()][x.ordinal()] = value;
		else matrix[x.ordinal()][y.ordinal()] = value;
	}

	public void calculate(int x, int y, Function<Double, Double> calculate) {
		double value;
		if(transposed) value = matrix[y][x];
		else value = matrix[x][y];

		set(x, y, calculate.apply(value));
	}

	/**
	 * this assumes the passed matrix is a compatible matrix and performs no checks on dimensions
	 * @param by the matrix to multiply by
	 * @return a this.getHeight X by.getWidth matrix
	 */
	public Matrix multiply(Matrix by) {
		Matrix result = new Matrix(getHeight(), by.getWidth());

		for (int x = 0; x < getHeight(); x++) {
			for(int y = 0; y < by.getWidth(); y++) {
				for(int col = 0; col < by.getHeight(); col++) {
					double toAdd = get(x, col) * by.get(col, y);
					result.calculate(x, y, cur -> cur + toAdd);
				}
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

		for(int x = 0; x < getWidth(); x++) {
			output.append("| ");
			for(int y = 0; y < getHeight(); y++) {
				if(y > 0) output.append(", ");
				output.append(formatToLengthFormat);

				// the current longest vs the current value to 2 decimal places
				longest = Math.max(longest, String.format("%.2f", get(x, y)).length());
				formatArgs[(getHeight() * x) + y] = get(x, y);
			}
			output.append(" |%%n");
		}
		char emDash = '\u2014';
		String padding = "" + emDash + emDash;
		String line = padding;
		for(int i = 0; i < getWidth(); i++) {
			for(int f = 0; f < longest; f++){
				line += emDash;
			}
			line += padding;
		}
		String format = String.format(output.toString(), longest);
		return line + '\n' + String.format(format, formatArgs) + line;
	}
}
