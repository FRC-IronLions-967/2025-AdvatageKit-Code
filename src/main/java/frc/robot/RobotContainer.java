// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MoveClimberInCommand;
import frc.robot.commands.MoveClimberOutCommand;
import frc.robot.commands.MoveWholeArmToPositionCommand;
import frc.robot.commands.RunAlgaeManipulatorCommand;
import frc.robot.commands.RunCoralManipulatorCommand;
import frc.robot.commands.ScoreAlgaeManipulatorCommand;
import frc.robot.commands.ToggleRatchetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  //   private final Vision vision;
  private final ArmSubsystem arm;
  private final ClimberSubsystem climber;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, VisionConstants.robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, VisionConstants.robotToCamera1),
        //         new VisionIOPhotonVision(
        //             objectDetectionCameraName, VisionConstants.robotToODcamera));
        arm = new ArmSubsystem();
        climber = new ClimberSubsystem();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             objectDetectionCameraName, robotToODcamera, drive::getPose));
        arm = new ArmSubsystem();
        climber = new ClimberSubsystem();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        arm = new ArmSubsystem();
        climber = new ClimberSubsystem();
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "L4Position",
        new MoveWholeArmToPositionCommand(arm, Constants.L4ElevatorPosition, Constants.L4ArmAngle));
    NamedCommands.registerCommand(
        "L3Position",
        new MoveWholeArmToPositionCommand(
            arm, Constants.L3ElevatorPosition, Constants.L2L3ArmAngle));
    NamedCommands.registerCommand(
        "L2Position",
        new MoveWholeArmToPositionCommand(
            arm, Constants.L2ElevatorPosition, Constants.L2L3ArmAngle));
    NamedCommands.registerCommand(
        "L3Algae",
        new MoveWholeArmToPositionCommand(
            arm, Constants.L3AlgaeElevatorPosition, Constants.reefAlgaeAngle));
    NamedCommands.registerCommand(
        "L2Algae",
        new MoveWholeArmToPositionCommand(
            arm, Constants.L3AlgaeElevatorPosition, Constants.reefAlgaeAngle));
    NamedCommands.registerCommand(
        "coralStationPosition",
        new MoveWholeArmToPositionCommand(
            arm, Constants.coralElevatorPosition, Constants.coralArmAngle));
    NamedCommands.registerCommand(
        "IntakeCoral", new RunCoralManipulatorCommand(arm, Constants.coralIntakeSpeed));
    NamedCommands.registerCommand(
        "IntakeAlgae", new RunAlgaeManipulatorCommand(arm, Constants.algaeIntakeSpeed));
    NamedCommands.registerCommand(
        "PlaceCoral", new RunCoralManipulatorCommand(arm, Constants.coralScoringSpeed));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Command intakeCoralCommand =
        new SequentialCommandGroup(
            new MoveWholeArmToPositionCommand(
                arm, Constants.coralElevatorPosition, Constants.coralArmAngle),
            new RunCoralManipulatorCommand(arm, Constants.coralIntakeSpeed));

    Command intakeAlgaeL3 =
        new SequentialCommandGroup(
            new MoveWholeArmToPositionCommand(
                arm, Constants.L3AlgaeElevatorPosition, Constants.reefAlgaeAngle),
            new RunAlgaeManipulatorCommand(arm, Constants.algaeIntakeSpeed));

    Command intakeAlgaeL2 =
        new SequentialCommandGroup(
            new MoveWholeArmToPositionCommand(
                arm, Constants.L2AlgaeElevatorPosition, Constants.reefAlgaeAngle),
            new RunAlgaeManipulatorCommand(arm, Constants.algaeIntakeSpeed));

    Command intakeAlgaeLolipop =
        new SequentialCommandGroup(
            new MoveWholeArmToPositionCommand(
                arm, Constants.processorElevatorPosition, Constants.processorAlgaeAngle),
            new RunAlgaeManipulatorCommand(arm, Constants.algaeIntakeSpeed));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Reset gyro to 0° when B button is pressed
    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driverController.a().onTrue(new MoveClimberOutCommand(climber));
    driverController.b().onTrue(new MoveClimberInCommand(climber));
    driverController.y().onTrue(new ToggleRatchetCommand(climber));

    operatorController.a().onTrue(new RunCoralManipulatorCommand(arm, Constants.coralScoringSpeed));
    operatorController.b().onTrue(intakeCoralCommand);
    operatorController
        .y()
        .onTrue(
            new MoveWholeArmToPositionCommand(
                arm, Constants.L4ElevatorPosition, Constants.L4ArmAngle));
    operatorController
        .x()
        .onTrue(
            new MoveWholeArmToPositionCommand(
                arm, Constants.L3ElevatorPosition, Constants.L2L3ArmAngle));
    operatorController
        .rightBumper()
        .onTrue(
            new MoveWholeArmToPositionCommand(
                arm, Constants.L2ElevatorPosition, Constants.L2L3ArmAngle));
    operatorController
        .povDown()
        .onTrue(new ScoreAlgaeManipulatorCommand(arm, Constants.algaeScoringSpeed));
    operatorController.povDown().onFalse(new ScoreAlgaeManipulatorCommand(arm, 0));
    operatorController.povRight().onTrue(intakeAlgaeL3);
    operatorController.povLeft().onTrue(intakeAlgaeL2);
    operatorController
        .povUp()
        .onTrue(
            new MoveWholeArmToPositionCommand(
                arm, Constants.bargeElevatorPosition, Constants.bargeAlgaeAngle));
    operatorController.leftBumper().onTrue(intakeAlgaeLolipop);
    operatorController
        .leftTrigger()
        .onTrue(new MoveWholeArmToPositionCommand(arm, 0, Constants.defaultArmAngle));
    operatorController
        .rightTrigger()
        .onTrue(new MoveWholeArmToPositionCommand(arm, 0, Constants.climbArmAngle));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
