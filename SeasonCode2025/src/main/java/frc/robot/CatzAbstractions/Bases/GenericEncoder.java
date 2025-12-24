package frc.robot.CatzAbstractions.Bases;

import java.util.function.DoubleSupplier;

public abstract class GenericEncoder {
	private final String name;
	private final DoubleSupplier rawRadiansSupplier; // optional supplier for absolute encoders
	protected boolean inverted = false;
	protected double offsetRadians = 0.0;

	// Use this when subclass overrides getRawRadiansImpl() (legacy behavior)
	public GenericEncoder(String name) {
		this.name = name;
		this.rawRadiansSupplier = null;
	}

	// Use this to support EncoderIOAbsolute-style implementations that provide a raw angle supplier
	public GenericEncoder(String name, DoubleSupplier rawRadiansSupplier) {
		this.name = name;
		this.rawRadiansSupplier = rawRadiansSupplier;
	}

	// Reads the raw absolute angle from the hardware in radians [0, 2π)
	// If a supplier is provided (EncoderIOAbsolute), use it; otherwise delegate to subclass
	protected double getRawRadians() {
		if (rawRadiansSupplier != null) {
			double val = rawRadiansSupplier.getAsDouble();
			return normalizeRadians(val);
		}
		return normalizeRadians(getRawRadiansImpl());
	}

	// Subclasses implement this when not using the supplier-based constructor
	protected double getRawRadiansImpl() {
		throw new UnsupportedOperationException("getRawRadiansImpl() not implemented and no supplier provided");
	}

	public String getName() {
		return name;
	}

	public void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public boolean isInverted() {
		return inverted;
	}

	public void setOffsetRadians(double offsetRadians) {
		this.offsetRadians = normalizeRadians(offsetRadians);
	}

	public double getOffsetRadians() {
		return offsetRadians;
	}

	// Angle with offset and inversion applied, wrapped to [0, 2π)
	public double getWrappedRadians() {
		double angle = getRawRadians();
		if (inverted) {
			angle = (2.0 * Math.PI) - angle;
		}
		angle = angle + offsetRadians;
		return normalizeRadians(angle);
	}

	// Angle with offset and inversion applied, wrapped to [0, 360)
	public double getWrappedDegrees() {
		return Math.toDegrees(getWrappedRadians());
	}

	// Rotations [0, 1)
	public double getWrappedRotations() {
		return getWrappedRadians() / (2.0 * Math.PI);
	}

	// Unwrapped angle in radians using the smallest difference from raw plus offset (may exceed 2π if offset > 2π)
	public double getRadians() {
		double angle = getRawRadians();
		if (inverted) {
			angle = (2.0 * Math.PI) - angle;
		}
		return angle + offsetRadians;
	}

	public double getDegrees() {
		return Math.toDegrees(getRadians());
	}

	public double getRotations() {
		return getRadians() / (2.0 * Math.PI);
	}

	// New methods to support position/velocity/reset/close

	// Default position in radians uses unwrapped angle calculation
	public double getPositionRadians() {
		return getRadians();
	}

	// Subclasses should override if hardware provides velocity
	public abstract double getVelocityRadiansPerSec();

	// Default reset just clears software offset; subclasses may also reset hardware position
	public abstract void reset(double offsetRadians);


	protected static double normalizeRadians(double angle) {
		angle = angle % (2.0 * Math.PI);
		if (angle < 0) {
			angle += 2.0 * Math.PI;
		}
		return angle;
	}
}
