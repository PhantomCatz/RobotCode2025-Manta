package frc.robot.CatzSubsystems.CatzWrist;

import frc.robot.Utilities.MotorUtil.Gains;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.LoggedTunableNumber;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.CatzConstants;

public class WristConstants {
    public static final boolean isWristDisabled = false;

    public static final double WRIST_MOTOR_GEAR_REDUCTION = 3.0; //TODO
    public static final int WRIST_MOTOR_ID = 50;
    public static final double WRIST_RETRACT = 0.0;
    public static final double WRIST_CATCH = 0.0;

    // Positive is winching in negative is winching out

    public static final Gains slot0_gains =
        switch (CatzConstants.getRobotType()) {
            //(100.0, 0.0, 0.0, 0.25, 0.12, 0.01, 0.0);
            case SN2 -> new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //TBD FOR GODSAKE
            case SN1 -> new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = slot0_gains.kP();
		FXConfig.Slot0.kD = slot0_gains.kD();
		FXConfig.Slot0.kS = slot0_gains.kS();
		FXConfig.Slot0.kG = slot0_gains.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.Feedback.SensorToMechanismRatio = 1;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = 40;
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerOpposeMain = new boolean[] {false, false};
		IOConfig.followerBuses = new String[] {"", ""};
		IOConfig.followerIDs = new int[] {41, 42};
		return IOConfig;
	}

    public final static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("Wrist/TunnablePosition", 1);
    public final static LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/kP", 0.17);
    public final static LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/kI", 0.0);
    public final static LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/kD", 0.0006);

    public final static LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/kS", 0);
    public final static LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/kV", 0);
    public final static LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/kA", 0);
}
