package frc.robot.CatzSubsystems.CatzWrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzArm.ArmIOSim;
// import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
// import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.WinchingState;
// import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.Utilities.LoggedTunableNumber;

import static frc.robot.CatzSubsystems.CatzWrist.WristConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class CatzWrist extends SubsystemBase {
  public static final CatzWrist Instance = new CatzWrist();

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  static double manualPow = 0;
  static boolean isManual = false;
  static final double MANUAL_SCALE = 2.0;
  static double position;
  static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("Wrist/TunnablePosition", 1);
  static LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/kP", 0.17);
  static LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/kI", 0.0);
  static LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/kD", 0.0006);

  static LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/kS", 0);
  static LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/kV", 0);
  static LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/kA", 0);

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
    if(isWristDisabled) { //Comes from Wrist Constants
      io = new WristIONull();
      System.out.println("Wrist Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new WristIOReal();
          System.out.println("Wrist Configured for Real");
        break;
        case REPLAY:
          io = new WristIOReal() {};
          System.out.println("Wrist Configured for Replayed simulation");
        break;
        case SIM:
          io = new WristIOSim();
          System.out.println("Arm Configured for Simulation");
        break;
        default:
          io = new WristIONull();
          System.out.println("Wrist Unconfigured");
        break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RealInputs/wrist", inputs);
    if (DriverStation.isDisabled()) {
      io.setPower(0.0);
      manualPow = 0.0;
      targetPosition = WristPosition.MANUAL;

    } else {
      if(isManual || targetPosition == WristPosition.FULL_MANUAL) {
        io.setPower(manualPow);
        System.out.println(manualPow);
        //System.out.println("full");
      } else if(targetPosition == WristPosition.MANUAL) {
        io.setPosition(position);
        // System.out.println("semi");
      } else if(targetPosition != WristPosition.MANUAL && targetPosition != WristPosition.FULL_MANUAL) {
        //System.out.println("Target + " + position);
        io.setPosition(position);
      } else {
        io.setPower(0.0);
      }

      // if(inputs.commandedOutput > 0.1) {
      //   CatzLED.Instance.setWristDirection(WinchingState.EXTENDING);
      // } else if(inputs.commandedOutput < -0.1) {
      //   CatzLED.Instance.setWristDirection(WinchingState.RETRACTING);
      // } else {
      //   CatzLED.Instance.setWristDirection(WinchingState.IDLE);
      // }
    }

    Logger.recordOutput("Wrist/targetPosition", position);
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
