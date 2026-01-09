package frc.robot.CatzSubsystems.CatzIntakeRollers;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.GenericSparkmaxIOReal.MotorIOSparkMaxConfig;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
import frc.robot.Utilities.MotorUtil.Gains;

public class RollerConstants {
    // TalonFX motor, double FL, Gains s0g, NeutralModeValue motorMode
    // public static final SparkMax RollerMotor = new SparkMax(3, MotorType.kBrushless );
    public static final double Final_Ratio = 1.0;
    public static final Gains s0g = new Gains(0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2); // placeholder ofc
    public static final Gains s1g = new Gains(0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2); // placeholder ofcpublic

	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = s0g.kP();
		FXConfig.Slot0.kD = s0g.kD();
		FXConfig.Slot0.kS = s0g.kS();
		FXConfig.Slot0.kG = s0g.kG();

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = 3;
		IOConfig.mainBus = "";
		IOConfig.followerConfig = getFXConfig()
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(false));
		IOConfig.followerOpposeMain = new boolean[] {false};
		IOConfig.followerBuses = new String[] {""};
		IOConfig.followerIDs = new int[] {};
		return IOConfig;
	}

	public static MotorIOSparkMaxConfig getSparkConfig() {
		MotorIOSparkMaxConfig iConfig = new MotorIOSparkMaxConfig();
		iConfig.mainID = 5;
		iConfig.followerIDs = new int[] {};
		iConfig.followerOpposeMain = new boolean[] {false};
		iConfig.invertMotor = false;
		iConfig.currentLimitAmps = 40;
		iConfig.gearRatio = 1.0;
		return iConfig;
	}


}
