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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final PathConstraints pathConstraints =
      new PathConstraints(3, 3, 540, 720, 12, false);

  public static final double kSecondsPerMinute = 60.0;
  public static final double kElevatorAnalogZeroOffset = 4.23;
  public static final double kArmZeroOffset = 1.961;

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final Pose3d kLeftRedStation = kTagLayout.getTagPose(1).get();
  public static final Pose3d kRightRedStation = kTagLayout.getTagPose(2).get();
  public static final Pose3d kLeftBlueStation = kTagLayout.getTagPose(13).get();
  public static final Pose3d kRightBlueStation = kTagLayout.getTagPose(12).get();

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  // Physical Constants
  public static final double elevatorGearRatio = 14.087; // taken from robot 2/1
  public static final double elevatorSprocketRadius = 0.8755; // inches
  public static final double armGearRatio = 103.448; // taken from CAD 1/28
  public static final double coralWheelRadius = 1.0;
  public static final double algaeWheelRadius = 1.0;
  // Speeds
  public static final double coralIntakeSpeed = -6500;
  public static final double coralScoringSpeed = 6500;
  public static final double algaeIntakeSpeed = -1;
  public static final double algaeScoringSpeed = 1;
  // Constraints
  public static final double armWiringMinConstraint = 0.0;
  public static final double armWiringMaxConstraint = 4.732;
  public static final double armFullRotationElevatorHeight = 5.0;
  public static final double armWithAlgaeFullRotationElevatorHeight = 23;
  public static final double emptyArmConstraintForAlgaeManipulatorAtE0 =
      2.734; // Shouldn't go past this without raising the elevator.
  public static final double armWithAlgaeMinConstraint = 1.551;
  public static final double armWithAlgaeMaxConstraint = 2.103;
  // State coral has no additional constraints.

  // Arm positions
  public static final double L2L3ArmAngle = 4.02;
  public static final double L4ArmAngle = 4.187;
  public static final double coralArmAngle = 0.6;
  public static final double reefAlgaeAngle = 1.710;
  public static final double bargeAlgaeAngle = 0.29;
  public static final double processorAlgaeAngle = 1.89;
  public static final double climbArmAngle = Math.PI / 2;
  public static final double defaultArmAngle = Math.PI;
  // Elevator Positions
  public static final double L2ElevatorPosition = 10.0;
  public static final double L3ElevatorPosition = 25.0;
  public static final double L4ElevatorPosition = 53.4;
  public static final double L2AlgaeElevatorPosition = 12;
  public static final double L3AlgaeElevatorPosition = 29;
  public static final double coralElevatorPosition = 22.5;
  public static final double bargeElevatorPosition = 53.0;
  public static final double climbElevatorPosition = 0.0;
  public static final double processorElevatorPosition = 1.0;
  // Climb Positions
  public static final double climberOutPosition = 140.0;
  public static final double climberInSpeed = -1;
  public static final double climberRatchetOnPosition = 0.75;
  public static final double climberRatchetOffPosition = 0.5;
  // Tolerances
  public static final double elevatorTolerance = 2.0;
  public static final double armTolerance = 0.1;
  public static final double climberTolerance = 5.0;
}
