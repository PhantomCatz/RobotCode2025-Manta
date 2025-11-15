package frc.robot.Bases;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import frc.robot.Utilities.MotorUtil.Gains;

public class TalonFXIOReal extends MotorIO {

    // initialize follower if needed?
    private TalonFX talonMotors[];

    private Gains slot0_gainsM;
    private Gains slot1_gainsM;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private double Final_Ratio;

    private final ControlRequestGetter requestGetter = new ControlRequestGetter();

    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    private Setpoint setpoint = Setpoint.withNeutralSetpoint();
    private boolean enabled = true;

    /**
     * basic
     * 1 motor
     * @param motor motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     * @param motorMode motor mode
     */
    public TalonFXIOReal(TalonFX motor, double FL, Gains s0g, Gains s1g,  NeutralModeValue motorMode) {
        super(1);
        talonMotors = new TalonFX[] {motor};

        Final_Ratio = FL;

        setMotorConfig(s0g, s1g, motorMode, false);
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
    public TalonFXIOReal(double FL, Gains s0g, Gains s1g, NeutralModeValue motorMode, boolean setFollow, TalonFX... motors) {
        super(motors.length);
        talonMotors = motors;
        Final_Ratio = FL;

        setMotorConfig(s0g, s1g, motorMode, setFollow);
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
        config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
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
                talonMotors[i].setControl(new Follower(talonMotors[0].getDeviceID(), false));
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
    
            inputs[i].motorRotations = motorPosition.getValueAsDouble() * Final_Ratio; //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
            inputs[i].velocityInchPerSec = angularVelocity.getValueAsDouble() * Final_Ratio;
            inputs[i].acceleration = angularAcceleration.getValueAsDouble() * Final_Ratio;
            inputs[i].appliedVoltage = appliedVoltage.getValueAsDouble();
            inputs[i].supplyCurrentAmps = supplyCurrent.getValueAsDouble();
            inputs[i].torqueCurrentAmps = torqueCurrent.getValueAsDouble();
            inputs[i].tempCelcius = motorTemp.getValueAsDouble();
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

    @Override
    public void setGainsSlot(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
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
        if (followerTalon == null) {
            talonMotors[0].setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
        else {
            talonMotors[0].setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            for (int i = 0; i < followerTalon.length; i++) {
                followerTalon[i].setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            }

        }
    }

    @Override
    public void runMotor(double speed) {
        // System.out.println(speed);
        talonMotors[0].setControl(new DutyCycleOut(speed));
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

    public final void applySetpoint(Setpoint setpointToApply) {
		setpoint = setpointToApply;
		if (enabled) {
			setpointToApply.apply(this);
		}
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
    public void runCurrent(double amps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runCurrent'");
    }

    @Override
    public void setGainsSlot(double kP, double kI, double kD) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGainsSlot'");
    }

    @Override
    public void runCharacterizationMotor(double input) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runCharacterizationMotor'");
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdleMode'");
    }

}
