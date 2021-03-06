/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import java.nio.file.Paths;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.Drive.TurnToAngleProfiled;
import frc.robot.commands.FloorIntake.LowerFloorIntake;
import frc.robot.commands.FloorIntake.RaiseFloorIntake;
import frc.robot.commands.FloorIntake.RunFloorIntake;
import frc.robot.commands.MultiSubsystemCommandGroups.RunBallPath;
import frc.robot.commands.MultiSubsystemCommandGroups.Shoot;
import frc.robot.commands.MultiSubsystemCommandGroups.ShootAllBalls;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.commands.Shooter.ShooterRPM;
import frc.robot.commands.Tower.EmptyTower;
import frc.robot.commands.Vision.AlignToGoalAuto;
import frc.robot.commands.Vision.AlignToGoalInPlace;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.Tower;
import frc.robot.tracking.LimelightInterface;
import frc.robot.subsystems.FloorIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RightSideAuto extends SequentialCommandGroup {
  /**
   * Creates a new RightSideAuto.
   */
  public RightSideAuto(Drive drive, Shooter shooter, Tower tower, Sorter sorter, FloorIntake intake,
      LimelightInterface limelight) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        new SequentialCommandGroup(
            new TurnToAngle(15.0, drive, () -> 0.0),
            new ParallelCommandGroup(new AlignToGoalAuto(drive),
                new ShooterRPM(shooter, tower, 
                () -> (!limelight.alignedToGoal()),
                // () -> true,
                true)),

            new ParallelCommandGroup(new EmptyTower(tower, shooter, sorter),
                new ShooterRPM(shooter,
                    tower,
                    () -> (tower.getBeamBreakFourState() || tower.getBeamBreakThreeState()
                        || tower.getBeamBreakTwoState() || tower.getBeamBreakOneState()),
                    // () -> true,
                    true))),

        new TurnToAngle(180.0, drive, () -> 0.0),

        new LowerFloorIntake(intake),

        new WaitCommand(.5),

        new ParallelRaceGroup(
            new RunBallPath(intake, sorter, tower),
            new TravelPath(Paths.get("/home/lvuser/LatestRightTrenchRun.wpilib.json"), drive, false)),

        new ParallelRaceGroup(
            new RunBallPath(intake, sorter, tower),
            new TravelPath(Paths.get("/home/lvuser/deploy/RightTrenchReturnRun.wpilib.json"), drive, true)   
        ),
        
    //   new RaiseFloorIntake(intake),

      new ParallelRaceGroup(
        new WaitCommand(.5),   
        new RunBallPath(intake, sorter, tower)
      ),
      new ParallelDeadlineGroup(
        new TurnToAngle(-165.0, drive, ()-> 0.0),
        new SetShooterSpeed(shooter, SmartDashboard.getNumber("Shooter/RPM Set Point", 4200))
      ),
    //   new LowerFloorIntake(intake),

      new SequentialCommandGroup(
            new ParallelCommandGroup(
                  new AlignToGoalAuto(drive), 
                  new ShooterRPM(
                    shooter, 
                    tower,
                    ()-> (!limelight.alignedToGoal()),
                    // () -> true,
                    true)),

            new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new ShooterRPM(
                    shooter,
                    tower,
                    () -> true,
                    true
                )
            ),

            new ParallelDeadlineGroup(
                new EmptyTower(tower, shooter, sorter),
                new RunFloorIntake(intake),
                new ShooterRPM(
                    shooter, 
                    tower,
                    ()-> true,
                        // (tower.getBeamBreakFourState() ||
                        // tower.getBeamBreakThreeState() ||
                        // tower.getBeamBreakTwoState()   || 
                        // tower.getBeamBreakOneState()), 
                        true))
        ),
        new InstantCommand(() -> drive.setCoastMode())
    );
  }
}
