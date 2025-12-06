package frc.robot.Bases;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.lang.module.ModuleReader;
import java.util.ArrayList;
import frc.robot.Utilities.MotorUtil.Gains;

public class SparkmaxIOReal extends MotorIO {
    // initialize follower if needed?

    private SparkMax[] sparks;

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
    public SparkmaxIOReal(SparkMax[] motors, double FL, Gains s0g, IdleMode motorMode) {

        this(motors, FL, s0g);

        config.idleMode(motorMode);

        sparks[0].getEncoder().setPosition(0);

        sparks[0].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        for (int i = 1; i < motors.length; i++) {
            SparkMaxConfig followerConfig = new SparkMaxConfig();
            followerConfig.apply(config);
            followerConfig.follow(sparks[0].getDeviceId(), false);
            sparks[i].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /**
     * basic, not done
     * 1 motor
     * @param leader motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     */
    public SparkmaxIOReal(SparkMax[] motors, double FL, Gains s0g) {
        super(1, Units.Rotations, Units.Seconds);
        sparks = motors;

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

        sparks[0].getEncoder().setPosition(0);

        sparks[0].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        for (int i = 1; i < motors.length; i++) {
            SparkMaxConfig followerConfig = new SparkMaxConfig();
            followerConfig.apply(config);
            followerConfig.follow(sparks[0].getDeviceId(), false);
            sparks[i].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        

    }

    public void updateInputs() {
        for (int i = 0; i < sparks.length; i++) {
            inputs[i].motorRotations = sparks[i].getAbsoluteEncoder().getPosition(); //TODO does this need to be multiplied by final ratio?
            inputs[i].velocityInchPerSec = sparks[i].getAbsoluteEncoder().getVelocity();
            inputs[i].appliedVoltage = sparks[i].getAppliedOutput();
            inputs[i].supplyCurrentAmps = sparks[i].getOutputCurrent();
            //inputs[i].torqueCurrentAmps = //TODO I have literally no idea
            inputs[i].tempCelcius = sparks[i].getMotorTemperature();
        }
    }

    @Override
    public void stop() {
        for (int i = 0; i < sparks.length; i++) {
            sparks[i].stopMotor();
        }
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD) {
        config.apply(new ClosedLoopConfig().pid(kP, kI, kD));
        for (int i = 0; i < sparks.length; i++) {
            sparks[i].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        for (int i = 0; i < sparks.length; i++) {
            sparks[i].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
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
    public void setFF(double kS, double kV, double kA) {
        // doesnt exist
    }

    @Override
    public void setPercentOutput(double percent) {
        // doesnt exist
    }

    @Override
    public void setPosition(double pos) {
        // doesnt exist
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        // doesnt exist
    }

    @Override
    public void setCoastOut() {
        // doesnt exist
    }

    @Override
    public void setNeutralOut() {
        // doesnt exist
    }

    @Override
    public void setCurrentPosition(Angle mechanismPosition) {
        // doesnt exist
    }

    @Override
    public void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {
        // doesnt exist
    }

    @Override
    public void setMotionMagicSetpoint(Angle mechanismPosition) {
        // doesnt exist
    }

    @Override
    public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
        // doesnt exist
    }

    @Override
    public void setDutyCycleSetpoint(Dimensionless percent) {
        // doesnt exist
    }

    @Override
    public void setPositionSetpoint(Angle mechanismPosition) {
        // doesnt exist
    }

    @Override
    public void setVoltageSetpoint(Voltage voltage) {
        // doesnt exist
    }

    @Override
    public void zeroSensors() {
        // doesnt exist
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        // doesnt exist
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
