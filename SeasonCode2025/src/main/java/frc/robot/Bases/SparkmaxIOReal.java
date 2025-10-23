// package frc.robot.Bases;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.controls.*;
// import com.ctre.phoenix6.signals.*;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import java.util.ArrayList;
// import java.util.concurrent.BlockingQueue;
// import java.util.concurrent.LinkedBlockingQueue;
// import java.util.concurrent.ThreadPoolExecutor;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Dimensionless;
// import edu.wpi.first.units.measure.Voltage;
// import frc.robot.Utilities.MotorUtil.Gains;;

// public class SparkmaxIOReal implements MotorIO {
//     // initialize follower if needed?
//     private SparkMax leaderSpark;
//     private ArrayList<SparkMax> followerSpark;

//     private Gains slot0_gainsM;
//     private Gains slot1_gainsM;

//     private final SparkMaxConfig config = new SparkMaxConfig();

//     private final StatusSignal<Angle> internalPositionRotations;
//     private final StatusSignal<AngularVelocity> velocityRps;
    

//     private double Final_Ratio;

//     private final ControlRequestGetter requestGetter = new ControlRequestGetter();

//     private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
//     private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

//     private boolean enabled = true;

//     /**
//      * basic, not done
//      * 1 motor
//      * @param motor motor
//      * @param FL Final Ratio
//      * @param s0g slot 0 gains
//      * @param motorMode motor mode
//      */
//     public SparkmaxIOReal(SparkMax motor, double FL, Gains s0g, NeutralModeValue motorMode) {

//         leaderSpark = motor;

//         Final_Ratio = FL;

//         slot0_gainsM = s0g;

//         internalPositionRotations = new Angle(leaderSpark.getAbsoluteEncoder().getPosition());
//         velocityRps = leaderSpark.getAbsoluteEncoder().getVelocity();   .degrees(leaderSpark.getAbsoluteEncoder().getPosition()
        

//         BaseStatusSignal.setUpdateFrequencyForAll(
//         100,
//         internalPositionRotations,
//         velocityRps
//         );

//         // PID configs
//         config.Slot0.kP = slot0_gainsM.kP();
//         config.Slot0.kI = slot0_gainsM.kI();
//         config.Slot0.kD = slot0_gainsM.kD();
        

//         config.Slot1.kS = slot1_gainsM.kS();
//         config.Slot1.kV = slot1_gainsM.kV();
//         config.Slot1.kA = slot1_gainsM.kA();
//         config.Slot1.kP = slot1_gainsM.kP();
//         config.Slot1.kI = slot1_gainsM.kI();
//         config.Slot1.kD = slot1_gainsM.kD();
//         config.Slot1.kG = slot1_gainsM.kG();


//         // Supply Current Limits
//         //config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
//         //config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
//         config.CurrentLimits.SupplyCurrentLimitEnable = true;
//         config.CurrentLimits.SupplyCurrentLimit = 100.0;
//         config.MotorOutput.NeutralMode = motorMode;

//         // Motion Magic Parameters

//         config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


//         //leaderSpark.setPosition(0);

//         leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     }

//     /**
//      * basic, not done
//      * 2 motors
//      * @param leader 1st motor
//      * @param followerMotor 2nd motor, automatically set as same direction as leader
//      * @param FL Final Ration
//      * @param s0g slot 0 gains
//      * @param s1g slot 1 gains
//      * @param motorMode motor mode
//      */
//     public SparkmaxIOReal(SparkMax leader, ArrayList<SparkMax> followerMotor, double FL, Gains s0g, Gains s1g, NeutralModeValue motorMode) {

//         leaderSpark = leader;
//         followerSpark = followerMotor;

//         Final_Ratio = FL;

//         slot0_gainsM = s0g;
//         slot1_gainsM = s1g;

//         internalPositionRotations = leaderSpark.getPosition();
//         velocityRps = leaderSpark.getVelocity();
        

//         BaseStatusSignal.setUpdateFrequencyForAll(
//         100,
//         internalPositionRotations,
//         velocityRps
//         );

//         // PID configs
//         config.Slot0.kS = slot0_gainsM.kS();
//         config.Slot0.kV = slot0_gainsM.kV();
//         config.Slot0.kA = slot0_gainsM.kA();
//         config.Slot0.kP = slot0_gainsM.kP();
//         config.Slot0.kI = slot0_gainsM.kI();
//         config.Slot0.kD = slot0_gainsM.kD();
//         config.Slot0.kG = slot0_gainsM.kG();

//         config.Slot1.kS = slot1_gainsM.kS();
//         config.Slot1.kV = slot1_gainsM.kV();
//         config.Slot1.kA = slot1_gainsM.kA();
//         config.Slot1.kP = slot1_gainsM.kP();
//         config.Slot1.kI = slot1_gainsM.kI();
//         config.Slot1.kD = slot1_gainsM.kD();
//         config.Slot1.kG = slot1_gainsM.kG();

//         // Supply Current Limits, does this need a varialbe input into it?
//         config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
//         config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
//         config.CurrentLimits.SupplyCurrentLimitEnable = true;
//         config.CurrentLimits.SupplyCurrentLimit = 80.0;
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


//         leaderSpark.setPosition(0);


//         leaderSpark.getConfigurator().apply(config, 1.0);

//         for (int i = 0; i < followerSpark.size(); i++) {
//             followerSpark.get(i).setPosition(0);
//             followerSpark.get(i).getConfigurator().apply(config, 1.0);
//             followerSpark.get(i).setControl(new Follower(leaderSpark.getDeviceID(), false));
//         }

//     }

//     /**
//      * basic, not done
//      * 1 motor
//      * @param leader motor
//      * @param FL Final Ratio
//      * @param s0g slot 0 gains
//      * @param s1g slot 1 gains
//      */
//     public SparkmaxIOReal(SparkMax motor, double FL, Gains s0g, Gains s1g) {

//         leaderSpark = motor;

//         Final_Ratio = FL;

//         slot0_gainsM = s0g;
//         slot1_gainsM = s1g;

//         internalPositionRotations = leaderSpark.getPosition();
//         velocityRps = leaderSpark.getVelocity();
        

        
//         // BaseStatusSignal.setUpdateFrequencyForAll(
//         // 100,
//         // internalPositionRotations,
//         // velocityRps,
//         // appliedVoltage.get(0),
//         // supplyCurrent.get(0),
//         // supplyCurrent.get(1),
//         // torqueCurrent.get(0),
//         // torqueCurrent.get(1),
//         // tempCelsius.get(0),
//         // tempCelsius.get(1));
    
        

//         // PID configs
//         config.Slot0.kS = slot0_gainsM.kS();
//         config.Slot0.kV = slot0_gainsM.kV();
//         config.Slot0.kA = slot0_gainsM.kA();
//         config.Slot0.kP = slot0_gainsM.kP();
//         config.Slot0.kI = slot0_gainsM.kI();
//         config.Slot0.kD = slot0_gainsM.kD();
//         config.Slot0.kG = slot0_gainsM.kG();

//         config.Slot1.kS = slot1_gainsM.kS();
//         config.Slot1.kV = slot1_gainsM.kV();
//         config.Slot1.kA = slot1_gainsM.kA();
//         config.Slot1.kP = slot1_gainsM.kP();
//         config.Slot1.kI = slot1_gainsM.kI();
//         config.Slot1.kD = slot1_gainsM.kD();
//         config.Slot1.kG = slot1_gainsM.kG();


//         // Supply Current Limits
//         config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
//         config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
//         config.CurrentLimits.SupplyCurrentLimitEnable = true;
//         config.CurrentLimits.SupplyCurrentLimit = 80.0;
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         // Motion Magic Parameters

//         config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


//         leaderSpark.setPosition(0);

//         leaderSpark.getConfigurator().apply(config, 1.0);

//     }

//     /**
//      * basic, not done
//      * 2 motors
//      * @param leader 1st motor
//      * @param followerMotor 2nd motor, automatically set as same direction as leader
//      * @param FL Final Ration
//      * @param s0g slot 0 gains
//      * @param s1g slot 1 gains
//      */
//     public SparkmaxIOReal(SparkMax leader, ArrayList<SparkMax> followerMotor, double FL, Gains s0g, Gains s1g) {

//         leaderSpark = leader;
//         followerSpark = followerMotor;

//         Final_Ratio = FL;

//         slot0_gainsM = s0g;
//         slot1_gainsM = s1g;

//         internalPositionRotations = leaderSpark.getPosition();
//         velocityRps = leaderSpark.getVelocity();
        

//         BaseStatusSignal.setUpdateFrequencyForAll(
//         100,
//         internalPositionRotations,
//         velocityRps
//         );

//         // PID configs
//         config.Slot0.kS = slot0_gainsM.kS();
//         config.Slot0.kV = slot0_gainsM.kV();
//         config.Slot0.kA = slot0_gainsM.kA();
//         config.Slot0.kP = slot0_gainsM.kP();
//         config.Slot0.kI = slot0_gainsM.kI();
//         config.Slot0.kD = slot0_gainsM.kD();
//         config.Slot0.kG = slot0_gainsM.kG();

//         config.Slot1.kS = slot1_gainsM.kS();
//         config.Slot1.kV = slot1_gainsM.kV();
//         config.Slot1.kA = slot1_gainsM.kA();
//         config.Slot1.kP = slot1_gainsM.kP();
//         config.Slot1.kI = slot1_gainsM.kI();
//         config.Slot1.kD = slot1_gainsM.kD();
//         config.Slot1.kG = slot1_gainsM.kG();

//         // Supply Current Limits, does this need a varialbe input into it?
//         config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
//         config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
//         config.CurrentLimits.SupplyCurrentLimitEnable = true;
//         config.CurrentLimits.SupplyCurrentLimit = 80.0;
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


//         leaderSpark.setPosition(0);


//         leaderSpark.getConfigurator().apply(config, 1.0);

//         for (int i = 0; i < followerSpark.size(); i++) {
//             followerSpark.get(i).setPosition(0);
//             followerSpark.get(i).getConfigurator().apply(config, 1.0);
//             followerSpark.get(i).setControl(new Follower(leaderSpark.getDeviceID(), false));
//         }

//     }

//     public void updateInputs(MotorIOInputs inputs) {
//         inputs.isLeaderMotorConnected =
//             BaseStatusSignal.refreshAll(
//                 internalPositionRotations,
//                 velocityRps
                

//             ).isOK();

//         // inputs.isFollowerMotorConnected = // TODO Some mechanisms may not have a followerer for their subtsystem rendering this redundant
//         //     BaseStatusSignal.refreshAll(
//         //         appliedVoltage.get(1),
//         //         supplyCurrent.get(1),
//         //         torqueCurrent.get(1),
//         //         tempCelsius.get(1))
//         //     .isOK();

//         inputs.motorRotations = internalPositionRotations.getValueAsDouble() * Final_Ratio; //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
//         inputs.velocityInchPerSec = velocityRps.getValueAsDouble() * Final_Ratio;

//     }

//     @Override
//     public void stop() {
//         leaderSpark.stopMotor();
//     }

//     @Override
//     public void setPosition(double pos) {
//         leaderSpark.setPosition(pos);
//     }

//     @Override
//     public void setGainsSlot0(double kP, double kI, double kD) {
//         config.apply(new ClosedLoopConfig().pid(kP, kI, kD));
//         leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     @Override
//     public void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
//         config.Slot1.kP = kP;
//         config.Slot1.kI = kI;
//         config.Slot1.kD = kD;
//         config.Slot1.kS = kS;
//         config.Slot1.kV = kV;
//         config.Slot1.kA = kA;
//         config.Slot1.kG = kG;
//         leaderSpark.getConfigurator().apply(config);
//     }

//     @Override
//     public void setBrakeMode(boolean enabled) {
//         if (followerSpark == null) {
//             config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
//             leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         }
//         else {
//             config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
//             for (int i = 0; i < followerSpark.size(); i++) {
//                 followerSpark.get(i).configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//             }
//             leaderSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
//         }
//     }

//     @Override
//     public void runMotor(double speed) {
//         System.out.println(speed);
//         leaderSpark.set(speed);
//     }

//     /* pretty sure this doesnt exist
//     @Override
//     public void setFF(double kS, double kV, double kA) {
//         config.Slot0.kS = kS;
//         config.Slot0.kV = kV;
//         config.Slot0.kA = kA;
//         leaderSpark.getConfigurator().apply(config);
//     }
//     */

//     public MotorIOInputs getMotorIOInputs() {
//         return new MotorIO.MotorIOInputs();
//     }

//     public static class ControlRequestGetter { // TODO pretty cool!
// 		public ControlRequest getVoltageRequest(Voltage voltage) {
// 			return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(false);
// 		}

// 		public ControlRequest getDutyCycleRequest(Dimensionless percent) {
// 			return new DutyCycleOut(percent.in(Units.Percent));
// 		}

// 		public ControlRequest getMotionMagicRequest(Angle mechanismPosition) {
// 			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
// 		}

// 		public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity) {
// 			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
// 		}

// 		public ControlRequest getPositionRequest(Angle mechanismPosition) {
// 			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
// 		}
// 	}

//     @Override
//     public void setIdleMode(IdleMode mode) {
//         config.idleMode(null);
//     }

//     /* I do not believe this exists i think perhaps maybe im not actually really sure but i can find it
//     private void setControl(ControlRequest request) {
// 		leaderSpark.setControl(request);
// 	}
    

//     @Override
// 	public void setNeutralOut() {
// 		setControl(new NeutralOut());
// 	}
    
// 	@Override
// 	public void setCoastOut() {
// 		setControl(new CoastOut());
// 	}

//     @Override
//     public void runPercentOutput(double percent) {
//         setControl(new DutyCycleOut(percent));
//     }
//     */
// 	@Override
// 	public void setCurrentPosition(Angle mechanismPosition) {
// 		threadPoolExecutor.submit(() -> {
// 			leaderSpark.setPosition(mechanismPosition);
// 		});
// 	}
    
// }