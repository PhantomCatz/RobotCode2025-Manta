package frc.robot.CatzSubsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.Utilities.EqualsUtil;

public class SubystemVisualizer {
    private static Pose3d[] simMechanismPoses = {new Pose3d(), new Pose3d(), new Pose3d()};

    public static final double firstStageHeight = Units.inchesToMeters(34.968750);
    public static final double stageHeight = Units.inchesToMeters(34.0);
    public static final double stageThickness = Units.inchesToMeters(1.0);
    public static final double outtakeToTop = Units.inchesToMeters(5.512529);
    public static final double outtakeToBottom = Units.inchesToMeters(7.25);
    public static final double stageToStage = Units.inchesToMeters(8.0);

    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(0.0);
    public static final Translation2d superstructureOrigin2d = new Translation2d(0.0825, 0.029);
    public static final Translation3d superstructureOrigin3d = new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());
    public static final Translation2d outtakeOrigin2d = superstructureOrigin2d.plus(new Translation2d(outtakeToBottom + stageThickness * 2, elevatorAngle));
    public static final Translation3d outtakeOrigin3d = new Translation3d(outtakeOrigin2d.getX(), 0.0, outtakeOrigin2d.getY());

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(
                                                        Units.feetToMeters(8.0),
                                                        Units.feetToMeters(8.0),
                                                        new Color8Bit(Color.kDarkGray)
                                                        );

    private final LoggedMechanismLigament2d elevatorMechanism;
    private final LoggedMechanismLigament2d giantPivotMechanism;
    private final LoggedMechanismLigament2d wristMechanism;
    private final String name;
    SubystemVisualizer(String name) {
        this.name = name;
        LoggedMechanismRoot2d root = mechanism.getRoot(
                                                name + " Root", superstructureOrigin2d.getX(), superstructureOrigin2d.getY()
        );
        giantPivotMechanism = root.append(
                                        new LoggedMechanismLigament2d(
                                                        name + " Pivot",
                                                        Units.inchesToMeters(1.0),
                                                        0.0,
                                                        8.0,
                                                        new Color8Bit(Color.kFirstRed))
        );
        elevatorMechanism = giantPivotMechanism.append(
                                        new LoggedMechanismLigament2d(
                                                        name + " Elevator",
                                                        Units.inchesToMeters(26.0),
                                                        elevatorAngle.getDegrees(),
                                                        4.0,
                                                        new Color8Bit(Color.kFirstBlue))
        );
        wristMechanism = elevatorMechanism.append(
                                                new LoggedMechanismLigament2d(
                                                        name + " Wrist",
                                                        Units.inchesToMeters(6.0),
                                                        90.0,
                                                        4.0,
                                                        new Color8Bit(Color.kOrange))
        );
    }

    public void update(double elevatorHeightMeters, Rotation2d giantPivotFinalAngle, Rotation2d wristAngle) {
        if (Robot.isSimulation()) {
                elevatorMechanism.setLength(
                        EqualsUtil.epsilonEquals(elevatorHeightMeters, 0.0)
                        ? Units.inchesToMeters(1.0)
                        : elevatorHeightMeters);
                giantPivotMechanism.setAngle(giantPivotFinalAngle);
                wristMechanism.setAngle(wristAngle);
                Logger.recordOutput("Mechanism2d/" + name, mechanism);
        }

        // Max of top of carriage or starting height
        final double heightFromBottom = elevatorHeightMeters + outtakeToBottom + outtakeToTop + stageThickness * 2.0;
        final double firstStageHeight = Math.max(heightFromBottom - SubystemVisualizer.firstStageHeight - stageThickness, stageThickness);
        final double secondStageHeight = Math.max(firstStageHeight - stageHeight + stageToStage, 0.0);

        Pose3d pivotPose3d = new Pose3d(
                                outtakeOrigin3d.plus(
                                        new Translation3d(
                                                elevatorHeightMeters, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
                                new Rotation3d(
                                        0.0,
                                        // Have to invert angle due to CAD??
                                        -giantPivotFinalAngle.getRadians(),
                                        0.0)
        );

        Logger.recordOutput("Visualization/FinalComponentPoses", simMechanismPoses);
        Logger.recordOutput(
                "Visualization/" + name + "/SubsystemStack",
                new Pose3d(
                        superstructureOrigin3d.plus(
                                new Translation3d(
                                        secondStageHeight, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
                        Rotation3d.kZero),
                new Pose3d(
                        superstructureOrigin3d.plus(
                                new Translation3d(
                                        firstStageHeight, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
                        Rotation3d.kZero),
                new Pose3d(pivotPose3d.getTranslation(), Rotation3d.kZero),
                pivotPose3d);
    }


    // TODO Possible depreciated code
    public static Pose3d getSimPose(int index) {
        return simMechanismPoses[index];
    }

    public static void setSimPose(int index, Pose3d pose) {
        simMechanismPoses[index] = pose;
    }
}
