// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivetrain.OIConstants;
import frc.robot.Constants.Drivetrain.DriveConstants;

import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint.DriveToPoint;
import frc.robot.commands.Drivetrain.Auto.Odometry.DriveToPoint.PathFollower;
import frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint.DriveToPoint_Vision;
import frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint.FollowTraj_NDT;
import frc.robot.commands.Drivetrain.Auto.Vision.DriveToPoint.PathFollower_Vision;
import frc.robot.commands.Drivetrain.Instants.ResetHeadingCommand;
import frc.robot.commands.Drivetrain.Tuner.SwerveDriveTunerCommand;
import frc.robot.subsystems.Drivetrain.AdvantageKit.SwerveDriveController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController _CommandDriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController _CommandOperatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  Trigger isLeftYPositive =  _CommandOperatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, DriveConstants.kDriveDeadband);
  Trigger isLeftYNegative =  _CommandOperatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, -DriveConstants.kDriveDeadband);


  private final CommandGenericHID operator_scorepad = new CommandGenericHID(2);



  /**
   * Substsyems
   */

//  ExampleSparkMaxSubsystem _armSubsystem;
  SwerveDriveController _driveSubsystem;

  SendableChooser<Command> _autoSelector = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setupSubsystems();
    setDefaultCommands();
    initCommandChooser();
    bindDriverController();
    bindOperatorController();
    bindOPscorepad();

//    NamedCommands.registerCommand();

  }

  private void setupSubsystems() {
    SubManager.initialize();
    _driveSubsystem = SubManager.get_drivetrain();
  }

  private void setDefaultCommands() {
    List<Pose2d> plist = new ArrayList<>();
    plist.add(new Pose2d(2,1,new Rotation2d()));
    plist.add(new Pose2d(5,3,new Rotation2d()));
    plist.add(new Pose2d(7,2,new Rotation2d()));


    SubManager.get_drivetrain().setDefaultCommand(
//            new DriveToPoint_Vision(new Pose2d())
//              new PathFollower_Vision(FollowTraj_NDT.convertToPoseList(new PathPlannerTrajectory(PathPlannerPath.fromPathFile("PickupGP3"), new ChassisSpeeds())))
//            new SwerveModuleTunerCommand(SubsystemContainer.get_drivetrain())
//
            new SwerveDriveCommand(
                    SubManager.get_drivetrain(),
                    m_driverController::getLeftX,
                    m_driverController::getLeftY,
                    m_driverController::getRightX,
                    m_driverController::getRightTriggerAxis
            )
    );

//    FollowTraj_NDT trajNotime = new FollowTraj_NDT("ScoreGP3");

  }

  private void bindDriverController() {

    Trigger a = _CommandDriverController.a();
      a.whileTrue(new TURBOSwerveDriveCommand(
              SubManager.get_drivetrain(),
              m_driverController::getLeftX,
              m_driverController::getLeftY,
              m_driverController::getRightX
      ));


    Trigger b = _CommandDriverController.b();
    b.whileTrue(new DriveToPoint(new Pose2d()));

    Trigger x = _CommandDriverController.x();
    x.onTrue(new ResetHeadingCommand(SubManager.get_drivetrain()));

    Trigger y = _CommandDriverController.y();


    Trigger lb = _CommandDriverController.leftBumper();
    lb.toggleOnTrue(new LockDriveCommand(SubManager.get_drivetrain()));


    Trigger rb = _CommandDriverController.rightBumper();


    Trigger ltr = _CommandDriverController.leftTrigger();


    Trigger rtr = _CommandDriverController.rightTrigger();


    Trigger dpadup = _CommandDriverController.povUp();
    //none

    Trigger dpadright = _CommandDriverController.povRight();

    Trigger dpaddown = _CommandDriverController.povDown();


    Trigger dpadleft = _CommandDriverController.povLeft();


    Trigger start = _CommandDriverController.start();
    //none

    Trigger back = _CommandDriverController.back();
    //none

    Trigger ljoybtn = _CommandDriverController.leftStick();

    Trigger rjoybtn = _CommandDriverController.rightStick();
    //none

  }

  private void bindOperatorController() {
    Trigger a = _CommandOperatorController.a();


    Trigger b = _CommandOperatorController.b();


    Trigger x = _CommandOperatorController.x();


    Trigger y = _CommandOperatorController.y();



    Trigger lb = _CommandOperatorController.leftBumper();



    Trigger rb = _CommandOperatorController.rightBumper();


    Trigger ltr = _CommandOperatorController.leftTrigger();

    Trigger rtr = _CommandOperatorController.rightTrigger();

    Trigger dpadup = _CommandOperatorController.povUp();

    Trigger dpadright = _CommandOperatorController.povRight();


    Trigger dpaddown = _CommandOperatorController.povDown();
    //none

    Trigger dpadleft = _CommandOperatorController.povLeft();


    Trigger start = _CommandOperatorController.start();

    Trigger back = _CommandOperatorController.back();



    Trigger ljoybtn = _CommandOperatorController.leftStick();
    //none

    Trigger rjoybtn = _CommandOperatorController.rightStick();
    //none

  }

  private void initCommandChooser(){


//    NamedCommands.registerCommand("BANANA", new playBananaMan());
//    NamedCommands.

//    _autoSelector.setDefaultOption("BANANA", new playBananaMan());

    _autoSelector.addOption("Nothing (15s)", new WaitCommand(15));
    _autoSelector.addOption("Say Hello", new PrintCommand("Hello There"));

    SmartDashboard.putData("Auto Selector", _autoSelector);
  }

  private void bindOPscorepad() {
    Trigger r1c1 = operator_scorepad.button(1);


    Trigger r1c2 = operator_scorepad.button(2);


    Trigger r1c3 = operator_scorepad.axisGreaterThan(3, 0.75);


    Trigger r2c1 = operator_scorepad.axisGreaterThan(2, 0.75);


    Trigger r2c2 = operator_scorepad.button(3);


    Trigger r2c3 = operator_scorepad.button(4);


    Trigger r3c1 = operator_scorepad.button(6);


    Trigger r3c2 = operator_scorepad.button(5);


    Trigger r3c3 = operator_scorepad.button(7);


    Trigger r4c1 = operator_scorepad.button(8);


    Trigger r4c2 = operator_scorepad.button(9);


    Trigger r4c3 = operator_scorepad.button(10);


  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SwerveDriveTunerCommand(SubManager.get_drivetrain());
//    return _autoSelector.getSelected();
  }
}
