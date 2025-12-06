package frc.robot.Bases;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Utilities.MotorUtil.Gains;

public class TalonFXSIOReal extends MotorIO {

    // initialize follower if needed?
    private TalonFXS talonMotors[];

    private Gains slot0_gainsM;
    private Gains slot1_gainsM;

    private final TalonFXSConfiguration config = new TalonFXSConfiguration();

    private final double FINAL_RATIO;

    private final ControlRequestGetter requestGetter = new ControlRequestGetter();

    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    private Setpoint setpoint = Setpoint.withNeutralSetpoint();
    private boolean enabled = true;

    private String name;

    /**
     * basic
     * 1 motor
     * @param motor motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     * @param motorMode motor mode
     */
    public TalonFXSIOReal(TalonFXS motor, double FL, Gains s0g, Gains s1g,  NeutralModeValue motorMode, String name) {
        super(1, Units.Rotations, Units.Seconds);
        talonMotors = new TalonFXS[] {motor};

        FINAL_RATIO = FL;

        setMotorConfig(s0g, s1g, motorMode, false);

        this.name = name;
    }

    /**
     * basic, not done
     * multiple motors, same control
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param motorMode motor mode
     */
    public TalonFXSIOReal(double FL, Gains s0g, Gains s1g, NeutralModeValue motorMode, boolean setFollow, String name, TalonFXS... motors) {
        super(motors.length, Units.Rotations, Units.Seconds);
        talonMotors = motors;
        FINAL_RATIO = FL;

        setMotorConfig(s0g, s1g, motorMode, setFollow);

        this.name = name;
    }

    private void setMotorConfig(Gains slot0Gains, Gains slot1Gains, NeutralModeValue motorMode, boolean setFollower){
        slot0_gainsM = slot0Gains;
        slot1_gainsM = slot1Gains;

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
        config.MotorOutput.NeutralMode = motorMode;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        for(int i = 0; i < talonMotors.length; i++){
            talonMotors[i].setPosition(0.0);
            talonMotors[i].getConfigurator().apply(config, 1.0);
        }

        if(setFollower){
            for(int i = 1; i < talonMotors.length; i++){
                talonMotors[i].setControl(new Follower(talonMotors[0].getDeviceID(), false)); //TODO make a custom talon config class that has an array that holds which motors will oppose the master
            }
        }
    }

    public void updateInputs() {

        for(int i = 0; i < talonMotors.length; i++){
            StatusSignal<Angle> motorPosition = talonMotors[i].getPosition();
            StatusSignal<AngularVelocity> angularVelocity = talonMotors[i].getVelocity();
            StatusSignal<AngularAcceleration> angularAcceleration = talonMotors[i].getAcceleration();
            StatusSignal<Current> supplyCurrent = talonMotors[i].getSupplyCurrent();
            StatusSignal<Current> torqueCurrent = talonMotors[i].getTorqueCurrent(); //TODO is this needed?
            StatusSignal<Voltage> appliedVoltage = talonMotors[i].getMotorVoltage();
            StatusSignal<Temperature> motorTemp = talonMotors[i].getDeviceTemp();

            inputs[i].isMotorConnected =
                BaseStatusSignal.refreshAll(
                    motorPosition,
                    angularVelocity,
                    angularAcceleration,
                    supplyCurrent,
                    torqueCurrent,
                    appliedVoltage,
                    motorTemp
                ).isOK();

            inputs[i].motorRotations = motorPosition.getValueAsDouble() * FINAL_RATIO; //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
            inputs[i].velocityInchPerSec = angularVelocity.getValueAsDouble() * FINAL_RATIO;
            inputs[i].acceleration = angularAcceleration.getValueAsDouble() * FINAL_RATIO;
            inputs[i].appliedVoltage = appliedVoltage.getValueAsDouble();
            inputs[i].supplyCurrentAmps = supplyCurrent.getValueAsDouble();
            inputs[i].torqueCurrentAmps = torqueCurrent.getValueAsDouble();
            inputs[i].tempCelcius = motorTemp.getValueAsDouble();

            Logger.processInputs("RealInputs/"+name, inputs[i]);
        }


    }

    @Override
    public void stop() {
        talonMotors[0].setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void setPosition(double pos) {
        talonMotors[0].setPosition(pos);
    }

    private void applyConfig(TalonFX talon, TalonFXConfiguration config){
        talon.getConfigurator().apply(config);
    }
    //TODO order the methods properly

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        talonMotors[0].getConfigurator().apply(config);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        for(int i = 0; i < talonMotors.length; i++){
            talonMotors[i].setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        talonMotors[0].getConfigurator().apply(config);
    }

    public MotorIOInputs getMotorIOInputs() {
        return new MotorIO.MotorIOInputs();
    }

    @Override
    public void setMotionMagicParameters(double vel, double accel, double jerk) {
        config.MotionMagic.MotionMagicCruiseVelocity = vel;
        config.MotionMagic.MotionMagicAcceleration = accel;
        config.MotionMagic.MotionMagicJerk = jerk;
        talonMotors[0].getConfigurator().apply(config);
    }

    //NOTE try to run these velocity and motion magic motor controls
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
		talonMotors[0].setControl(request);
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
    public void setPercentOutput(double percent) {
        setControl(new DutyCycleOut(percent));
    }

    @Override
	public void setVoltageSetpoint(Voltage voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	@Override
	public void setDutyCycleSetpoint(Dimensionless percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	@Override
	public void setMotionMagicSetpoint(Angle mechanismPosition) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
	}

	@Override
	public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	@Override
	public void setPositionSetpoint(Angle mechanismPosition) {
		setControl(requestGetter.getPositionRequest(mechanismPosition));
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			talonMotors[0].setPosition(mechanismPosition);
		});
	}

    //NOTE fill these overrides out



    @Override
    public void setGainsSlot0(double kP, double kI, double kD) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGainsSlot'");
    }


    @Override
    public void setIdleMode(IdleMode mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdleMode'");
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroSensors'");
    }

    @Override
    public double getVelocityInch() {
        return inputs[0].velocityInchPerSec;
    }

    @Override
    public double getPositionInch() {
        return inputs[0].absoluteEncoderPositionRads;
    }

    @Override
    public double getSupplyCurrent() {
        return inputs[0].supplyCurrentAmps;
    }

    @Override
    public double getAcceleration() {
        return inputs[0].acceleration;
    }

    @Override
    public double getAppliedVoltage() {
        return inputs[0].appliedVoltage;
    }

    @Override
    public double getTemp() {
        return inputs[0].tempCelcius;
    }

    @Override
    public double getRotations() {
        return inputs[0].motorRotations;
    }

    @Override
    public AngularVelocity getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }

    @Override
    public Angle getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void useSoftLimits(boolean enable) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useSoftLimits'");
    }

}
