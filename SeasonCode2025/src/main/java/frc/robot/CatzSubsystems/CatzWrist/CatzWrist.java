package frc.robot.CatzSubsystems.CatzWrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

import static frc.robot.CatzSubsystems.CatzWrist.WristConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;

public class CatzWrist extends GenericMotorSubsystem<WristIO, WristIO.WristIOInputs> {
  private static final WristIO io = getIOInstance();
  private static final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  static WristIO getIOInstance() {
    switch (CatzConstants.hardwareMode) {
        case REAL:
            System.out.println("Wrist Configured for Real");
            return new WristIOTalonFX(WristConstants.getIOConfig());
        case SIM:
            System.out.println("Wrist Configured for Simulation");
            return new WristIOSim();
        default:
            System.out.println("Wrist Unconfigured");
            return new WristIOSim();
    }
}

  public static final CatzWrist Instance = new CatzWrist();

  static double manualPow = 0;
  static boolean isManual = false;
  static final double MANUAL_SCALE = 2.0;
  static double position;

  @RequiredArgsConstructor
  public enum WristPosition { //In Rotations //TODO not real working numbers
    RETRACT(() -> -100), //TBD
    HOME(() -> 0), //TBD
    EXTENDING(() -> 100), //TBD
    MANUAL(() -> 0.0),
    FULL_MANUAL(() -> 0.0),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }

  private WristPosition targetPosition = WristPosition.HOME;

  private CatzWrist() {
    super(io, inputs, "CatzWrist");
  }

  public Command Wrist_Home() {
    return runOnce(() -> setWristPos(WristPosition.HOME));
  }

  public Command fullWrist() {
    return runOnce(() -> setWristPos(WristPosition.RETRACT));
  }

  public Command extendWrist() {
    return runOnce(() -> setWristPos(WristPosition.EXTENDING));
  }

  public Command Wrist_Tunnable() {
    return runOnce(() -> setWristPos(WristPosition.TUNNABLE));
  }

  public void setWristPos(WristPosition target) {
    position = target.getTargetMotionPosition();
    targetPosition = target;
  }

  public double getWristPos() {
    return inputs.position;
  }

  public void wristSemiManual(double manualSemiPwr) {
    double previousPos = position;
    position += manualSemiPwr * MANUAL_SCALE;
    if (Math.abs(manualSemiPwr) < 0.1) {
      position = previousPos;
    }
    // System.out.println(position);
    targetPosition = WristPosition.MANUAL;
  }

  public void wristFullManual(double joystickPower) {
    manualPow = joystickPower;
    targetPosition = WristPosition.FULL_MANUAL;
  }

  public Command WristManualMode(Supplier<Double> manualSupplier) {
    return run(() -> wristFullManual(manualSupplier.get())).alongWith(Commands.print("hi"));
  }

  public Command CancelWrist() {
    Command cancel = Commands.runOnce(()->manualPow = 0.0);
    cancel.addRequirements(this);
    return cancel;
  }

  public void setOverrides(Supplier<Boolean> booleanSupplier) {
    isManual = booleanSupplier.get();
  }

}
