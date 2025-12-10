package frc.robot.CatzAbstractions.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.Setpoint;

public class GenericTalonFXIOReal implements GenericMotorIO {

    // initialize follower if needed
    private TalonFX leaderTalon;
    private TalonFX[] followerTalons;

    private Gains slot0_gainsM;
    private Gains slot1_gainsM;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<Angle> internalPositionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final List<StatusSignal<Voltage>> appliedVoltage;
    private final List<StatusSignal<Current>> supplyCurrent;
    private final List<StatusSignal<Current>> torqueCurrent;
    private final List<StatusSignal<Temperature>> tempCelsius;

    private final ControlRequestGetter requestGetter = new ControlRequestGetter();

    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    private Setpoint setpoint = Setpoint.withNeutralSetpoint();
    private boolean enabled = true;

    private static double Final_Ratio;

    /**
     * base for constructors
     * 1 motor sets bare minimum to not kill itself
     * User must set MotionMagic, current limits, etc after instantiation
     * @param leader motor
     * @param s0g slot 0 gains
     */
    public GenericTalonFXIOReal(double FL, Gains s0g, TalonFX motor) {

        leaderTalon = motor;
        Final_Ratio = FL;
        slot0_gainsM = s0g;

        internalPositionRotations = leaderTalon.getPosition();
        velocityRps = leaderTalon.getVelocity();
        acceleration = leaderTalon.getAcceleration();
        appliedVoltage = List.of(leaderTalon.getMotorVoltage());
        supplyCurrent = List.of(leaderTalon.getSupplyCurrent());
        torqueCurrent = List.of(leaderTalon.getTorqueCurrent());
        tempCelsius = List.of(leaderTalon.getDeviceTemp());


        // PID configs
        config.Slot0.kS = slot0_gainsM.kS();
        config.Slot0.kV = slot0_gainsM.kV();
        config.Slot0.kA = slot0_gainsM.kA();
        config.Slot0.kP = slot0_gainsM.kP();
        config.Slot0.kI = slot0_gainsM.kI();
        config.Slot0.kD = slot0_gainsM.kD();
        config.Slot0.kG = slot0_gainsM.kG();

        // Current Limits
        config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


        leaderTalon.getConfigurator().apply(config, 1.0);

    }

    /**
     * basic, not done
     * 1 motor
     * @param motor motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     * @param motorMode motor mode
     */
    public GenericTalonFXIOReal(double FL, Gains s0g, NeutralModeValue motorMode, TalonFX motor) {

        this(FL, s0g, motor);

        config.MotorOutput.NeutralMode = motorMode;
        leaderTalon.getConfigurator().apply(config, 1.0); // re-apply because other constructor has to go first

    }

    /**
     * basic, not done
     * 2 motors
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param motorMode motor mode
     */
    public GenericTalonFXIOReal(double FL, Gains s0g, NeutralModeValue motorMode, TalonFX... motors) {

        this(FL, s0g, motorMode, motors[0]);
        followerTalons = motors;

        for (int i = 1; i < followerTalons.length; i++) { // Skip first motor
            followerTalons[i].setPosition(0);
            followerTalons[i].getConfigurator().apply(config, 1.0);
            followerTalons[i].setControl(new Follower(leaderTalon.getDeviceID(), false));
        }

    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.isLeaderConnected =
            BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                acceleration,
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
            .isOK();

        if(followerTalons != null) {
            inputs.isFollowerConnected =
                BaseStatusSignal.refreshAll(
                    appliedVoltage.get(1),
                    supplyCurrent.get(1),
                    torqueCurrent.get(1),
                    tempCelsius.get(1))
                .isOK();
        }

        inputs.absoluteEncoderPosition = internalPositionRotations.getValueAsDouble() * Final_Ratio; //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
        inputs.velocityRPS = velocityRps.getValueAsDouble() * Final_Ratio;
        inputs.accelerationRPS = acceleration.getValueAsDouble() * Final_Ratio;
        inputs.appliedVolts = appliedVoltage.stream()
                                            .mapToDouble(StatusSignal::getValueAsDouble)
                                            .toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
        inputs.torqueCurrentAmps = torqueCurrent.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
        inputs.tempCelcius = tempCelsius.stream()
                                        .mapToDouble(StatusSignal::getValueAsDouble)
                                        .toArray();


    }

    @Override
    public void stop() {
        leaderTalon.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        leaderTalon.getConfigurator().apply(config, 1.0);
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        leaderTalon.getConfigurator().apply(config, 1.0);
    }

    @Override
    public void setGainsSlot1(double kP, double kI, double kD) {
        config.Slot1.kP = kP;
        config.Slot1.kI = kI;
        config.Slot1.kD = kD;
        leaderTalon.getConfigurator().apply(config, 1.0);
    }

    @Override
    public void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        config.Slot1.kP = kP;
        config.Slot1.kI = kI;
        config.Slot1.kD = kD;
        config.Slot1.kS = kS;
        config.Slot1.kV = kV;
        config.Slot1.kA = kA;
        config.Slot1.kG = kG;
        leaderTalon.getConfigurator().apply(config, 1.0);
    }

    /**
     * Sets the brake mode for the motor controllers. When brake mode is enabled,
     * the motor controllers will actively resist motion when no power is applied.
     * When disabled (coast mode), the motor controllers will allow the motors to
     * spin freely.
     *
     * @param enabled If true, sets the motor controllers to brake mode. If false,
     *                sets the motor controllers to coast mode.
     */
    @Override
    public void setBrakeMode(boolean enabled) {
        if (followerTalons == null) {
            leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
        else {
            leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            for (int i = 0; i < followerTalons.length; i++) {
                followerTalons[i].setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            }

        }
    }

    @Override
    public void runMotor(double speed) {
        // System.out.println(speed);
        leaderTalon.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        leaderTalon.getConfigurator().apply(config, 1.0);
    }

    @Override
    public void setMotionMagicParameters(double vel, double accel, double jerk) {
        config.MotionMagic.MotionMagicCruiseVelocity = vel;
        config.MotionMagic.MotionMagicAcceleration = accel;
        config.MotionMagic.MotionMagicJerk = jerk;
        leaderTalon.getConfigurator().apply(config, 1.0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        config.MotorOutput.NeutralMode = mode;
    }


    private void setControl(ControlRequest request) {
		leaderTalon.setControl(request);
	}


    @Override
    public void runPercentOutput(double percent) {
        setControl(new DutyCycleOut(percent));
    }

    public final void applySetpoint(Setpoint setpointToApply) {
		setpoint = setpointToApply;
		if (enabled) {
			setpointToApply.apply(this);
		}
	}

    @Override
	public void setVoltageSetpoint(double voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	@Override
	public void setDutyCycleSetpoint(double percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	@Override
	public void setMotionMagicSetpoint(double mechanismPosition) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
	}

	@Override
	public void setVelocitySetpoint(double mechanismVelocity) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	@Override
	public void setPositionSetpoint(double mechanismPosition) {
		setControl(requestGetter.getPositionRequest(mechanismPosition));
	}

	@Override
	public void setCurrentPosition(double mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			leaderTalon.setPosition(mechanismPosition);
		});
	}


    public static class ControlRequestGetter {
		public ControlRequest getVoltageRequest(double voltage) {
			return new VoltageOut(voltage);
		}

		public ControlRequest getDutyCycleRequest(double percent) {
			return new DutyCycleOut(percent);
		}

		public ControlRequest getMotionMagicRequest(double mechanismPosition) {
			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
		}

		public ControlRequest getVelocityRequest(double mechanismVelocity) {
			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
		}

		public ControlRequest getPositionRequest(double mechanismPosition) {
			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
		}
	}

}
