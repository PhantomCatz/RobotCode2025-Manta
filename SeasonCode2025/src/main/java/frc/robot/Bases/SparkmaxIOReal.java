package frc.robot.Bases;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.ArrayList;
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

        this(motor, FL, s0g);

        config.idleMode(motorMode);

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

        this(leader, FL, s0g, motorMode);

        followerSpark = followerMotor;

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
        this(leader, FL, s0g);

        followerSpark = followerMotor;

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
        inputs.appliedVoltage = leaderSpark.getAppliedOutput();
        inputs.supplyCurrentAmps = leaderSpark.getOutputCurrent();
        inputs.torqueCurrentAmps = //TODO I have literally no idea
        inputs.tempCelcius = leaderSpark.getMotorTemperature();
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

    @Override
    public void setIdleMode(IdleMode mode) {
        config.idleMode(mode);
    }

}
