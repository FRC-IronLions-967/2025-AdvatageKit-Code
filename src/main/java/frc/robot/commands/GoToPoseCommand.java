// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToPoseCommand extends Command {

  private Drive drive;

  private Pose2d pose;
  private List<Waypoint> pathWaypoints;
  private PathPlannerPath pathToDestination;

  /** Creates a new GoToPoseCommand. */
  public GoToPoseCommand(Drive drive, Pose2d pose) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    this.pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathWaypoints = PathPlannerPath.waypointsFromPoses(drive.getPose(), pose);
    pathToDestination =
        new PathPlannerPath(
            pathWaypoints, Constants.pathConstraints, null, new GoalEndState(0, null));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoBuilder.followPath(pathToDestination);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
