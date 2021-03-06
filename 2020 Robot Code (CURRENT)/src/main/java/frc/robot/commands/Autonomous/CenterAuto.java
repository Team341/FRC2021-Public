/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import java.nio.file.Paths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FloorIntake.LowerFloorIntake;
import frc.robot.commands.MultiSubsystemCommandGroups.RunBallPath;
import frc.robot.commands.Shooter.ShooterRPM;
import frc.robot.commands.Tower.EmptyTower;
import frc.robot.commands.Vision.AlignToGoalAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.Tower;
import frc.robot.tracking.LimelightInterface;
import frc.robot.subsystems.FloorIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterAuto extends SequentialCommandGroup {
  /**
   * Creates a new CenterAuto.
   */
  public CenterAuto(Drive drive, Shooter shooter, Tower tower, Sorter sorter, FloorIntake intake,
      LimelightInterface limelight) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        new SequentialCommandGroup(
            new ParallelCommandGroup(new AlignToGoalAuto(drive),
                new ShooterRPM(shooter, tower, () -> (!limelight.alignedToGoal()), false)),

            new ParallelCommandGroup(new EmptyTower(tower, shooter, sorter),
                new ShooterRPM(shooter,
                    tower,
                    () -> (tower.getBeamBreakFourState() || tower.getBeamBreakThreeState()
                        || tower.getBeamBreakTwoState() || tower.getBeamBreakOneState()),
                    false))),

        new FollowPath(Paths.get("/home/lvuser/deploy/BackupFromLine.wpilib.json"), drive, true)
            .getDriveForwardRamseteCommand(),

        new LowerFloorIntake(intake),

        new ParallelRaceGroup(
            new RunBallPath(intake, sorter, tower),
            new FollowPath(Paths.get("/home/lvuser/deploy/ScoopBalls.wpilib.json"), drive, false).getDriveForwardRamseteCommand()),

      new SequentialCommandGroup(
            new ParallelCommandGroup(
                  new AlignToGoalAuto(drive), 
                  new ShooterRPM(
                    shooter, 
                    tower,
                    ()->
                      (!limelight.alignedToGoal()), false)),

            new ParallelCommandGroup(
                  new EmptyTower(tower, shooter, sorter),
                  new ShooterRPM(
                    shooter, 
                    tower,
                    ()->
                      (tower.getBeamBreakFourState() ||
                      tower.getBeamBreakThreeState() ||
                      tower.getBeamBreakTwoState()   || 
                      tower.getBeamBreakOneState()), false))
        )
    );
  }
}
