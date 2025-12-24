package frc.robot.CatzAbstractions.io;

import frc.robot.CatzAbstractions.Bases.GenericEncoder;
import com.ctre.phoenix6.hardware.CANcoder;

public class EncoderIOAbsolute extends GenericEncoder {
	private static final double TWO_PI = 2.0 * Math.PI;

	private final CANcoder cancoder;
	private double offsetRadians = 0.0;

	/**
	 * Construct with a CANcoder device ID.
	 * @param name logical name for this encoder
	 * @param deviceId CAN ID of the CANcoder
	 */
	public EncoderIOAbsolute(String name, int deviceId) {
		super(name);
		this.cancoder = new CANcoder(deviceId);
	}

	/**
	 * Construct with CANcoder device ID and a position offset in radians.
	 * @param name logical name for this encoder
	 * @param deviceId CAN ID of the CANcoder
	 * @param offsetRadians offset applied to reported position (useful for zeroing)
	 */
	public EncoderIOAbsolute(String name, int deviceId, double offsetRadians) {
		super(name);
		this.cancoder = new CANcoder(deviceId);
		this.offsetRadians = offsetRadians;
	}

	@Override
	public double getPositionRadians() {
		// Phoenix 6 CANcoder position is reported in rotations
		double rotations = cancoder.getPosition().getValueAsDouble();
		return (rotations * TWO_PI) - offsetRadians;
	}

	@Override
	public double getVelocityRadiansPerSec() {
		// Phoenix 6 CANcoder velocity is reported in rotations per second
		double rotPerSec = cancoder.getVelocity().getValueAsDouble();
		return rotPerSec * TWO_PI;
	}

	@Override
	public void reset(double offset) {
		// Set current position to zero (rotations), keeping hardware absolute as reference
		cancoder.setPosition(offset);
		offsetRadians = offset;
	}

}
