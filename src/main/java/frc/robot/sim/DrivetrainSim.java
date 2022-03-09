// package frc.robot.sim;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.PWMSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;

// import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import org.photonvision.SimVisionSystem;
// import org.photonvision.SimVisionTarget;

// /**
//  * Implementation of a simulation of robot physics, sensors, motor controllers Includes a Simulated
//  * PhotonVision system and one vision target.
//  *
//  * <p>This class and its methods are only relevant during simulation. While on the real robot, the
//  * real motors/sensors/physics are used instead.
//  */
// public class DrivetrainSim {
//     // Simulated Motor Controllers
//     PWMSim leftLeader = new PWMSim(0);
//     PWMSim rightLeader = new PWMSim(1);
//     // private final WPI_TalonSRX m_intakeArmMotor = new WPI_TalonSRX(5);
//     // TalonSRXSimCollection m_intakeArmMotorSim = new TalonSRXSimCollection(m_intakeArmMotor);
//     // Simulation Physics
//     // Configure these to match your drivetrain's physical dimensions
//     // and characterization results.
//     LinearSystem<N2, N2, N2> drivetrainSystem =
//             LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3, 1.0);
//     DifferentialDrivetrainSim drivetrainSimulator =
//             new DifferentialDrivetrainSim(
//                     drivetrainSystem,
//                     DCMotor.getCIM(2),
//                     8,
//                     Units.feetToMeters(2.0),
//                     Units.inchesToMeters(6.0 / 2.0),
//                     null);
//     // SingleJointedArmSim m_intakeArmSim = 
//     //         new SingleJointedArmSim(
//     //         DCMotor.getFalcon500(1), 
//     //         1, 
//     //         0.2, 
//     //         Units.inchesToMeters(40), 
//     //         Units.degreesToRadians(-30), 
//     //         Units.degreesToRadians(110), 
//     //         4.53592, 
//     //         true);
//     // Simulated Vision System.
//     // Configure these to match your PhotonVision Camera,
//     // pipeline, and LED setup.
//     double camDiagFOV = 170.0; // degrees - assume wide-angle camera
//     //double camPitch = Units.radiansToDegrees(Robot.CAMERA_PITCH_RADIANS); // degrees
//     /double camHeightOffGround = Robot.CAMERA_HEIGHT_METERS; // meters
//     double maxLEDRange = 20; // meters
//     int camResolutionWidth = 640; // pixels
//     int camResolutionHeight = 480; // pixels
//     double minTargetArea = 10; // square pixels

//     SimVisionSystem simVision =
//             new SimVisionSystem(
//                     "photonvision",
//                     camDiagFOV,
//                     camPitch,
//                     new Transform2d(),
//                     camHeightOffGround,
//                     maxLEDRange,
//                     camResolutionWidth,
//                     camResolutionHeight,
//                     minTargetArea);

//     // See
//     // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
//     // page 208
//     double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
//     // See
//     // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
//     // page 197
//     double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
//     // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
//     // pages 4 and 5
//     double tgtXPos = Units.feetToMeters(54);
//     double tgtYPos =
//             Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
//     double newtgtXPos = 7.046;
//     double newtgtYPos = 3.751;
//     Pose2d farTargetPose = new Pose2d(new Translation2d(tgtXPos, tgtYPos), new Rotation2d(0.0));


//     Field2d field = new Field2d();

//     public DrivetrainSim() {
//         simVision.addSimVisionTarget(
//                 new SimVisionTarget(farTargetPose, Robot.TARGET_HEIGHT_METERS, targetWidth, targetHeight));
//         SmartDashboard.putData("Field", field);
//     }

//     /**
//      * Perform all periodic drivetrain simulation related tasks to advance our simulation of robot
//      * physics forward by a single 20ms step.
//      */
//     public void update() {
//         double leftMotorCmd = 0;
//         double rightMotorCmd = 0;
//         //double intakeMotorCmd = 0;
//         if (DriverStation.isEnabled() && !RobotController.isBrownedOut()) {
//             leftMotorCmd = leftLeader.getSpeed();
//             rightMotorCmd = rightLeader.getSpeed();
//             //intakeMotorCmd = m_intakeArmMotorSim.getMotorOutputLeadVoltage();
//         }

//         drivetrainSimulator.setInputs(
//                 leftMotorCmd * RobotController.getInputVoltage(),
//                 -rightMotorCmd * RobotController.getInputVoltage());
//         drivetrainSimulator.update(0.02);

//         // m_intakeArmSim.setInputVoltage(intakeMotorCmd);
//         // m_intakeArmSim.update(0.02);
//         // Update PhotonVision based on our new robot position.
//         simVision.processFrame(drivetrainSimulator.getPose());
//         field.setRobotPose(drivetrainSimulator.getPose());
//     }

//     /**
//      * Resets the simulation back to a pre-defined pose Useful to simulate the action of placing the
//      * robot onto a specific spot in the field (IE, at the start of each match).
//      *
//      * @param pose
//      */
//     public void resetPose(Pose2d pose) {
//         drivetrainSimulator.setPose(pose);
//     }
// }