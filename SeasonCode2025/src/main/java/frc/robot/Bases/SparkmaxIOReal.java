package frc.robot.Bases;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.ArrayList;
import frc.robot.Utilities.MotorUtil.Gains;

public class SparkmaxIOReal extends MotorIO {
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
    public void setGainsSlot(double kP, double kI, double kD) {
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

    //NOTE fill these overrides out

    @Override
    public void runCurrent(double amps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runCurrent'");
    }

    @Override
    public void setGainsSlot(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGainsSlot'");
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFF'");
    }

    @Override
    public void runCharacterizationMotor(double input) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runCharacterizationMotor'");
    }

    @Override
    public void setPercentOutput(double percent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runPercentOutput'");
    }

    @Override
    public void setPosition(double pos) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setNeutralMode'");
    }

    @Override
    public void setCoastOut() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCoastOut'");
    }

    @Override
    public void setNeutralOut() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setNeutralOut'");
    }

    @Override
    public void setCurrentPosition(Angle mechanismPosition) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentPosition'");
    }

    @Override
    public void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicParameters'");
    }

    @Override
    public void setMotionMagicSetpoint(Angle mechanismPosition) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicSetpoint'");
    }

    @Override
    public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocitySetpoint'");
    }

    @Override
    public void setDutyCycleSetpoint(Dimensionless percent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDutyCycleSetpoint'");
    }

    @Override
    public void setPositionSetpoint(Angle mechanismPosition) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPositionSetpoint'");
    }

    @Override
    public void setVoltageSetpoint(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltageSetpoint'");
    }

    @Override
    public void applySetpoint(Setpoint setpointToApply) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'applySetpoint'");
    }

}
