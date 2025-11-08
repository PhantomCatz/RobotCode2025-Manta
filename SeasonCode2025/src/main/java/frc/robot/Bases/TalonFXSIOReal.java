package frc.robot.Bases;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Utilities.MotorUtil.Gains;;

public class TalonFXSIOReal implements MotorIO {

    // initialize follower if needed?
    private TalonFXS leaderTalon;
    private ArrayList<TalonFXS> followerTalon;

    private Gains slot0_gainsM;
    private Gains slot1_gainsM;

    private final TalonFXSConfiguration config = new TalonFXSConfiguration();

    private final StatusSignal<Angle> internalPositionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Temperature> tempCelsius;

    private double Final_Ratio;

    private final ControlRequestGetter requestGetter = new ControlRequestGetter();

    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    /**
     * basic, not done
     * 1 motor
     * @param motor motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     * @param motorMode motor mode
     */
    public TalonFXSIOReal(TalonFXS motor, double FL, Gains s0g, Gains s1g, NeutralModeValue motorMode) {

        this(motor, FL, s0g, s1g);

        config.MotorOutput.NeutralMode = motorMode;

        leaderTalon.getConfigurator().apply(config, 1.0);

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
    public TalonFXSIOReal(TalonFXS leader, ArrayList<TalonFXS> followerMotor, double FL, Gains s0g, Gains s1g, NeutralModeValue motorMode) {

        this(leader, FL, s0g, s1g, motorMode);

        for (int i = 0; i < followerTalon.size(); i++) {
            followerTalon.get(i).setPosition(0);
            followerTalon.get(i).getConfigurator().apply(config, 1.0);
            followerTalon.get(i).setControl(new Follower(leaderTalon.getDeviceID(), false));
        }

    }

    /**
     * basic, not done
     * 1 motor
     * @param leader motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     */
    public TalonFXSIOReal(TalonFXS motor, double FL, Gains s0g, Gains s1g) {

        leaderTalon = motor;

        Final_Ratio = FL;

        slot0_gainsM = s0g;
        slot1_gainsM = s1g;

        internalPositionRotations = leaderTalon.getPosition();
        velocityRps = leaderTalon.getVelocity();
        acceleration = leaderTalon.getAcceleration();
        supplyCurrent = leaderTalon.getSupplyCurrent();
        torqueCurrent = leaderTalon.getTorqueCurrent();
        appliedVoltage = leaderTalon.getMotorVoltage();
        tempCelsius = leaderTalon.getDeviceTemp();


        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            internalPositionRotations,
            velocityRps
        );

        // PID configs
        config.Slot0.kS = slot0_gainsM.kS();
        config.Slot0.kV = slot0_gainsM.kV();
        config.Slot0.kA = slot0_gainsM.kA();
        config.Slot0.kP = slot0_gainsM.kP();
        config.Slot0.kI = slot0_gainsM.kI();
        config.Slot0.kD = slot0_gainsM.kD();
        config.Slot0.kG = slot0_gainsM.kG();

        config.Slot1.kS = slot1_gainsM.kS();
        config.Slot1.kV = slot1_gainsM.kV();
        config.Slot1.kA = slot1_gainsM.kA();
        config.Slot1.kP = slot1_gainsM.kP();
        config.Slot1.kI = slot1_gainsM.kI();
        config.Slot1.kD = slot1_gainsM.kD();
        config.Slot1.kG = slot1_gainsM.kG();


        // Supply Current Limits
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motion Magic Parameters

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


        leaderTalon.setPosition(0);

        leaderTalon.getConfigurator().apply(config, 1.0);

    }

    /**
     * basic, not done
     * 2 motors
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     */
    public TalonFXSIOReal(TalonFXS leader, ArrayList<TalonFXS> followerMotor, double FL, Gains s0g, Gains s1g) {

        this(leader, FL, s0g, s1g);

        for (int i = 0; i < followerTalon.size(); i++) {
            followerTalon.get(i).setPosition(0);
            followerTalon.get(i).getConfigurator().apply(config, 1.0);
            followerTalon.get(i).setControl(new Follower(leaderTalon.getDeviceID(), false));
        }

    }

    public void updateInputs(MotorIOInputs inputs) {
        inputs.isLeaderMotorConnected =
            BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                acceleration,
                supplyCurrent,
                torqueCurrent,
                appliedVoltage,
                tempCelsius


            ).isOK();

        inputs.motorRotations = internalPositionRotations.getValueAsDouble() * Final_Ratio; //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
        inputs.velocityInchPerSec = velocityRps.getValueAsDouble() * Final_Ratio;
        inputs.acceleration = acceleration.getValueAsDouble() * Final_Ratio;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelcius = tempCelsius.getValueAsDouble();


    }

    @Override
    public void stop() {
        leaderTalon.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void setPosition(double pos) {
        leaderTalon.setPosition(pos);
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
        leaderTalon.getConfigurator().apply(config);
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
        leaderTalon.getConfigurator().apply(config);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (followerTalon == null) {
            leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
        else {
            leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            for (int i = 0; i < followerTalon.size(); i++) {
                followerTalon.get(i).setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            }

        }
    }

    @Override
    public void runMotor(double speed) {
        // System.out.println(speed);
        leaderTalon.set(speed);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        leaderTalon.getConfigurator().apply(config);
    }

    public MotorIOInputs getMotorIOInputs() {
        return new MotorIO.MotorIOInputs();
    }

    public static class ControlRequestGetter { // TODO pretty cool!
		public ControlRequest getVoltageRequest(Voltage voltage) {
			return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(false);
		}

		public ControlRequest getDutyCycleRequest(Dimensionless percent) {
			return new DutyCycleOut(percent.in(Units.Percent));
		}

		public ControlRequest getMotionMagicRequest(Angle mechanismPosition) {
			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
		}

		public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity) {
			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
		}

		public ControlRequest getPositionRequest(Angle mechanismPosition) {
			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
		}
	}

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        config.MotorOutput.NeutralMode = mode;
    }


    private void setControl(ControlRequest request) {
		leaderTalon.setControl(request);
	}

    @Override
	public void setNeutralOut() {
		setControl(new NeutralOut());
	}

	@Override
	public void setCoastOut() {
		setControl(new CoastOut());
	}

    @Override
    public void runPercentOutput(double percent) {
        setControl(new DutyCycleOut(percent));
    }

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			leaderTalon.setPosition(mechanismPosition);
		});
	}

}
