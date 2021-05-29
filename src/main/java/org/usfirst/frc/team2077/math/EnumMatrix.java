package org.usfirst.frc.team2077.math;

import java.util.function.Function;

/**
 * A Wrapper around {@link Matrix} but provides better type support for use with Enum's
 *
 * Provides better type support for Matrix operations using <strong>enum</strong> prefixed functions
 * @param <Width> Enum to use for X indices
 * @param <Height> Enum to use for Y indices
 */
public class EnumMatrix<
	Width extends Enum<? super Width>,
	Height extends Enum<? super Height>
> extends Matrix {

	public EnumMatrix(Class<? super Width> width, Class<? super Height> height) {
		super(height.getEnumConstants().length, width.getEnumConstants().length);
	}

	protected EnumMatrix(EnumMatrix<Height, Width> toTranspose) {
		super(toTranspose);
	}

	public EnumMatrix(double[][] matrix) {
		super(matrix);
	}

	/**
	 * gets the value at ({@code x}, {@code y})
	 * @param x {@link Width} enum to get
	 * @param y {@link Height} enum to get
	 * @return the value at ({@code x}, {@code y})
	 */
	public double get(Enum<? super Width> x, Enum<? super Height> y) {
		return get(x.ordinal(), y.ordinal());
	}

	/**
	 * sets the {@code value} at ({@code x}, {@code y})
	 * @param x {@link Width} enum to set
	 * @param y {@link Height} enum to set
	 * @param value updated value
	 */
	public void set(Enum<? super Width> x, Enum<? super Height> y, double value) {
		set(x.ordinal(), y.ordinal(), value);
	}

	/**
	 * Assumes non-null arguments. Type compliant form of {@link #calculate(int, int, Function)}
	 * @param x {@link Width} enum to set
	 * @param y {@link Height} enum to set
	 * @param calculate function that takes the current value at ({@code x}, {@code y}) and returns an updated value
	 */
	public void calculate(Enum<? super Width> x, Enum<? super Height> y, Function<Double, Double> calculate) {
		set(x, y, calculate.apply(get(x, y)));
	}

	/**
	 * An {@link EnumMatrix} type compliant form of {@link #transpose()}
	 * @return A transposed copy of {@code this}
	 */
	public EnumMatrix<Height, Width> enumTranspose() {
		return new EnumMatrix<>(this);
	}

	/**
	 * Matrix multiplication. Assumes that {@code OtherHeight} and {@code OtherWidth} are equivalent to
	 * {@code Width} and {@code Height}, respectively.
	 * @param by matrix to multiply by
	 * @param <OtherHeight> {@code by}'s Height enum
	 * @param <OtherWidth> {@code by}'s Width enum
	 * @return the produce of {@code this} and {@code by}
	 */
	public <
		OtherHeight extends Enum<? super OtherHeight>,
		OtherWidth extends Enum<? super OtherWidth>
	> EnumMatrix<OtherWidth, Height> enumMultiply(EnumMatrix<OtherWidth, OtherHeight> by) {
		return multiply(by.matrix, EnumMatrix::new);
	}

	/**
	 * An {@link EnumMatrix} type compliant {@link #inverse3x3()} wrapper
	 * @return an inverted 3x3 form of {@code this}
	 */
	public EnumMatrix<Width, Height> enumInvert3x3() {
		return inverse3x3(EnumMatrix::new);
	}
}
