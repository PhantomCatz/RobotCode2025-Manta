package frc.robot.Bases;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Utilities.MotorUtil.Gains;;

public class SparkmaxIOReal implements MotorIO {
    // initialize follower if needed?
    private SparkMax leaderSpark;
    private ArrayList<SparkMax> followerSpark;

    private final SparkMaxConfig config = new SparkMaxConfig();

    // private double Final_Ratio; //TODO do we need this?

    /**
     * basic, not done
     * 1 motor
     * @param motor motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     * @param motorMode motor mode
     */
    public SparkmaxIOReal(SparkMax motor, double FL, Gains s0g, IdleMode motorMode) {

        leaderSpark = motor;

        //Final_Ratio = FL;

        // PID configs
        config.apply(new ClosedLoopConfig().pid(s0g.kP(), s0g.kI(), s0g.kD()));


        // Supply Current Limits
        config.smartCurrentLimit(100);
        config.idleMode(motorMode);

        // Motion Magic Parameters

        config.inverted(false); //is this supposed to be inverted? i just left it as false


        //leaderSpark.setPosition(0); I do not think you can set positions for sparkmax

        leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * basic, not done
     * 2 motors
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param motorMode motor mode
     */
    public SparkmaxIOReal(SparkMax leader, ArrayList<SparkMax> followerMotor, double FL, Gains s0g, IdleMode motorMode) {

        leaderSpark = leader;
        followerSpark = followerMotor;

        //Final_Ratio = FL;

        // PID configs
        config.apply(new ClosedLoopConfig().pid(s0g.kP(), s0g.kI(), s0g.kD()));

        // Supply Current Limits, does this need a varialbe input into it?
        config.smartCurrentLimit(100);
        config.idleMode(motorMode);

        config.inverted(false);


        // leaderSpark.setPosition(0);


        leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        for (int i = 0; i < followerSpark.size(); i++) {
            // followerSpark.get(i).setPosition(0);
            SparkMaxConfig followerConfig = new SparkMaxConfig();
            followerConfig.apply(config);
            followerConfig.follow(leaderSpark.getDeviceId(), false);
            followerSpark.get(i).configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

    }

    /**
     * basic, not done
     * 1 motor
     * @param leader motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     */
    public SparkmaxIOReal(SparkMax motor, double FL, Gains s0g) {

        leaderSpark = motor;

        // Final_Ratio = FL;

        // PID configs
        config.apply(new ClosedLoopConfig().pid(s0g.kP(), s0g.kI(), s0g.kD()));

        // Supply Current Limits
        // config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
        // config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.smartCurrentLimit(100);
        config.idleMode(IdleMode.kBrake);

        // Motion Magic Parameters

        config.inverted(false);


        // leaderSpark.setPosition(0);

        leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * basic, not done
     * 2 motors
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param FL Final Ration
     * @param s0g slot 0 gains
     */
    public SparkmaxIOReal(SparkMax leader, ArrayList<SparkMax> followerMotor, double FL, Gains s0g) {

        leaderSpark = leader;
        followerSpark = followerMotor;

        // Final_Ratio = FL;

        // PID configs
        config.apply(new ClosedLoopConfig().pid(s0g.kP(), s0g.kI(), s0g.kD()));

        // Supply Current Limits, does this need a varialbe input into it?
        config.smartCurrentLimit(100);
        config.idleMode(IdleMode.kBrake);

        config.inverted(false);


        // leaderSpark.setPosition(0);


        leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        for (int i = 0; i < followerSpark.size(); i++) {
            // followerSpark.get(i).setPosition(0);
            SparkMaxConfig followerConfig = new SparkMaxConfig();
            followerConfig.apply(config);
            followerConfig.follow(leaderSpark.getDeviceId(), false);
            followerSpark.get(i).configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

    }

    public void updateInputs(MotorIOInputs inputs) {
        inputs.motorRotations = leaderSpark.getAbsoluteEncoder().getPosition(); //TODO does this need to be multiplied by final ratio?
        inputs.velocityInchPerSec = leaderSpark.getAbsoluteEncoder().getVelocity();

    }

    @Override
    public void stop() {
        leaderSpark.stopMotor();
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD) {
        config.apply(new ClosedLoopConfig().pid(kP, kI, kD));
        leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (followerSpark == null) {
            config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
            leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        else {
            config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
            for (int i = 0; i < followerSpark.size(); i++) {
                followerSpark.get(i).configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }
            leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        }
    }

    @Override
    public void runMotor(double speed) {
        System.out.println(speed);
        leaderSpark.set(speed);
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
    public void setIdleMode(IdleMode mode) {
        config.idleMode(mode);
    }

}
