package org.usfirst.frc.team2077.math;

import java.util.function.Function;

public class EnumMatrix<
	Height extends Enum<? super Height>,
	Width extends Enum<? super Width>
> extends Matrix {

	public EnumMatrix(Class<? super Height> height, Class<? super Width> width) {
		super(height.getEnumConstants().length, width.getEnumConstants().length);
	}

	protected EnumMatrix(EnumMatrix<Width, Height> toTranspose) {
		super(toTranspose);
	}

	public EnumMatrix(double[][] matrix) {
		super(matrix);
	}

	public double get(Enum<? super Width> x, Enum<? super Height> y) {
		return get(x.ordinal(), y.ordinal());
	}

	public void set(Enum<? super Width> x, Enum<? super Height> y, double value) {
		set(x.ordinal(), y.ordinal(), value);
	}

	public void calculate(Enum<? super Width> x, Enum<? super Height> y, Function<Double, Double> calculate) {
		set(x.ordinal(), y.ordinal(), calculate.apply(get(x.ordinal(), y.ordinal())));
	}

	public EnumMatrix<Width, Height> enumTranspose() {
		return new EnumMatrix<>(this);
	}

	public <
		OtherHeight extends Enum<? super OtherHeight>,
		OtherWidth extends Enum<? super OtherWidth>
	> EnumMatrix<Height, OtherWidth> enumMultiply(EnumMatrix<OtherHeight, OtherWidth> by) {
		return multiply(by.matrix, EnumMatrix::new);
	}

	public EnumMatrix<Height, Width> enumInvert3x3() {
		return inverse3x3(EnumMatrix::new);
	}
}
