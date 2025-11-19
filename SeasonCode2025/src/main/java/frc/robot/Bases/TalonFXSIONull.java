package frc.robot.Bases;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXSIONull extends MotorIO {

    public TalonFXSIONull() {
        super(0);
    }

    @Override
    public void updateInputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroSensors'");
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGainsSlot0'");
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGainsSlot0'");
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFF'");
    }

    @Override
    public void setPercentOutput(double percent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPercentOutput'");
    }

    @Override
    public void setPosition(double pos) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setBrakeMode'");
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setNeutralMode'");
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdleMode'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
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
    public double getVelocityInch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocityInch'");
    }

    @Override
    public double getPositionInch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPositionInch'");
    }

    @Override
    public double getSupplyCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSupplyCurrent'");
    }}
