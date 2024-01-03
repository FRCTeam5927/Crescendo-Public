// package frc.robot.subsystems;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.*;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.libs.DiagnosticTable;

// public class ArmMechanism2D extends SubsystemBase {
//     Mechanism2d mechCanvas = new Mechanism2d(4,4);
//     MechanismRoot2d mechRoot = mechCanvas.getRoot("arm", 2, 2);
// //    MechanismObject2d
//     MechanismLigament2d arm = new MechanismLigament2d("arm", 0.7, 90);

//     DCMotor simMotor = DCMotor.getNEO(1).withReduction(25);
//     SingleJointedArmSim physArm = new SingleJointedArmSim(
//                     simMotor,
//                     25,
//                     500,
//                     0.7847,
//                     Units.degreesToRadians(-75),
//                     Units.degreesToRadians(255),
//                     false,
//                     VecBuilder.fill(Units.degreesToRadians(0.03)) // Add noise with a std-dev of 1 tick
//             );
//     public ArmMechanism2D() {
//         mechRoot.append(arm);
//         SmartDashboard.putData("sim_mech", mechCanvas);

// //        SmartDashboard.putData("mechroot", arm);

//     }

//     XboxController controller = new XboxController(0);


//     @Override
//     public void periodic() {

//         if(controller.getAButton()) {
//             setPosition(Math.PI);
//         } else {
//             physArm.setInputVoltage(0);
//         }

//     }


//     DiagnosticTable dtab = new DiagnosticTable("armSim");
//     PIDController pidController = new PIDController(100,0,0);
//     public void setPosition(double position) {
//         double output = pidController.calculate(physArm.getAngleRads(), position);
//         physArm.setInputVoltage(output);
// //        SmartDashboard.putNumber("current", physArm.getCurrentDrawAmps());
//         dtab.putNumber("Goal Position", position);
//         dtab.putNumber("Current Position", physArm.getAngleRads());
//         dtab.putNumber("PID Output", output);
//         dtab.putNumber("PID Error", pidController.getPositionError());
//         dtab.putNumber("current draw", physArm.getCurrentDrawAmps());

//         arm.setAngle(Units.radiansToDegrees(physArm.getAngleRads()));
//         physArm.update(0.02);
//     }
// }
