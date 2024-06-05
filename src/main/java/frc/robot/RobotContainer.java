// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmAutoAim;
import frc.robot.commands.ArmPIDCMD;
//import frc.robot.commands.ArmDownCMD;
import frc.robot.commands.ArmUpCMD;
import frc.robot.commands.AutoMoveCMD;
import frc.robot.commands.GyroResetCMD;
import frc.robot.commands.IntakeCMD;
import frc.robot.commands.IntakeCMD_NoBeamBreak;
import frc.robot.commands.IntakeOutCMD;
import frc.robot.commands.ShooterCMD;
import frc.robot.commands.ShooterCMD2;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.limelightInterface;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.ArrayList;
import java.util.List;

import javax.sql.XAConnectionBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem(intakeConstants.intakeSpeed);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ArmSubsystem m_arm_up = new ArmSubsystem();

  // The driver's controller
  //XboxController xboxController0 = new XboxController(OIConstants.kDriverControllerPort0);
  Joystick joystick1 = new Joystick(OIConstants.kDriverControllerPort1);
  Joystick joystick0 = new Joystick(OIConstants.kDriverControllerPort0);
  XboxController xboxController2 =new XboxController(OIConstants.kDriverControllerPort2);

  SendableChooser<Command> m_chooser = new SendableChooser();


   public  TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    public Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        new ArrayList<Translation2d>(),
        //List.of(new Translation2d(0.5, 0), new Translation2d(0.7, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.2, 0, new Rotation2d(0)),
        config);

    public Trajectory rotateRed = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(-60 * Math.PI / 180)),
        // Pass through these two interior waypoints, making an 's' curve path
        new ArrayList<Translation2d>(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(-60 * Math.PI / 180)),
        config);

    public Trajectory rotateBlue = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(60 * Math.PI / 180)),
        // Pass through these two interior waypoints, making an 's' curve path
        new ArrayList<Translation2d>(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, Math.sqrt(3), new Rotation2d(60 * Math.PI / 180)),
        config);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Auto Chooser",m_chooser);
    m_chooser.setDefaultOption("Front Subwoofer", FrontSubwoofer);
    m_chooser.addOption("Side Subwoofer", SideSubwoofer);
    m_chooser.addOption("Blue Side", SideMoveBlue);
    m_chooser.addOption("Red Side", SideMoveRed);
    m_chooser.addOption("Mobility", Mobility);
    m_chooser.addOption("No Auto", NoAuto);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(joystick0.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick0.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick1.getX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
        new JoystickButton(joystick1, 5)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));
        final JoystickButton Intake = new JoystickButton(joystick0, 1);  
        Intake.toggleOnTrue(new IntakeCMD( m_intake, true ));
        

        final JoystickButton outIntake = new JoystickButton(joystick0, 3);        
        outIntake.onTrue(new IntakeOutCMD( m_intake, true ));
        outIntake.onFalse(new IntakeOutCMD( m_intake, false ));

        final JoystickButton altIntake = new JoystickButton(joystick0, 6);        
        altIntake.whileTrue(new IntakeCMD_NoBeamBreak( m_intake, true ));


        final JoystickButton slowShooter = new JoystickButton(joystick1, 3);        
        slowShooter.onTrue(new ShooterCMD( m_shooter, m_intake, true, shooterConstants.shooterSpeed));

        /*final JoystickButton testshoot = new JoystickButton(joystick1, 3);
        testshoot.onTrue(new ParallelCommandGroup(
            new ShooterCMD2(m_shooter, true, shooterConstants.shooterSpeed),
            new SequentialCommandGroup(
                new WaitCommand(.7),
                new IntakeCMD_NoBeamBreak(m_intake, true)
            )
        ));
        testshoot.onTrue(new SequentialCommandGroup(
            new WaitCommand(1.2),
            new ParallelCommandGroup(
                new IntakeCMD_NoBeamBreak(m_intake, false),
                new ShooterCMD2(m_shooter, false, 0)
            )
        ));*/

        final JoystickButton fastShooter = new JoystickButton(joystick1, 1);        
        fastShooter.onTrue(new ShooterCMD( m_shooter, m_intake, true, shooterConstants.fastShooterSpeed));

        final JoystickButton rightArmUp = new JoystickButton(joystick1, 6);        
        rightArmUp.onTrue(new ArmUpCMD( m_arm_up, true, false ));
        rightArmUp.onFalse(new ArmUpCMD( m_arm_up, false, false ));

        final JoystickButton leftArmDown = new JoystickButton(joystick0, 5);        
        leftArmDown.onTrue(new ArmUpCMD( m_arm_up, false, true ));
        leftArmDown.onFalse(new ArmUpCMD( m_arm_up, false, false ));

        final JoystickButton gyroReset = new JoystickButton(joystick1, 11);
        //gyroReset.onTrue(new GyroResetCMD(m_robotDrive, 0));

        final JoystickButton JoyArmIntake = new JoystickButton(joystick0, Joystick.ButtonType.kTop.value);
        JoyArmIntake.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.NormIZone));

        final JoystickButton ArmAuto = new JoystickButton(joystick1, Joystick.ButtonType.kTop.value);
        ArmAuto.onTrue(new ArmAutoAim(m_arm_up, false, 0));

        final JoystickButton JoyArmStart = new JoystickButton(joystick1, 12);
        JoyArmStart.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kStartPos + 90, false, ArmConstants.NormIZone));

        final JoystickButton SENDIT = new JoystickButton(joystick0, 11);
        SENDIT.onTrue(new ShooterCMD(m_shooter, m_intake, true, shooterConstants.SENDIT));

        final JoystickButton aArmIntake = new JoystickButton(xboxController2, XboxController.Button.kA.value);
        aArmIntake.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.NormIZone));

        final JoystickButton xArmShoot = new JoystickButton(xboxController2, XboxController.Button.kX.value);
        xArmShoot.onTrue(new ArmPIDCMD(m_arm_up,  ArmConstants.kSpeaker + 90, false, ArmConstants.NormIZone));

        final JoystickButton bArmIntake2 = new JoystickButton(xboxController2, XboxController.Button.kB.value);
        bArmIntake2.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 89.5, false, ArmConstants.AutoShortIZone));

        final JoystickButton startArmStart = new JoystickButton(xboxController2, XboxController.Button.kStart.value);
        startArmStart.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kStartPos + 90, false, ArmConstants.NormIZone));

        final JoystickButton R1ArmPodium = new JoystickButton(xboxController2, XboxController.Button.kRightBumper.value);
        R1ArmPodium.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kPodium + 90, false, ArmConstants.NormIZone));

        final JoystickButton yArmStartLine = new JoystickButton(xboxController2, XboxController.Button.kY.value);
        yArmStartLine.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kStartLine + 90, false, ArmConstants.NormIZone));

        final JoystickButton L1ArmWingLine = new JoystickButton(xboxController2, XboxController.Button.kLeftBumper.value);
        L1ArmWingLine.onTrue(new ArmPIDCMD(m_arm_up, ArmConstants.kSideSubwoofer + 90, false, ArmConstants.NormIZone));


    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

private Command FrontSubwoofer =
     new SequentialCommandGroup(
        //new ArmPIDCMD(m_arm_up, ArmConstants.kSpeaker + 90, false, ArmConstants.NormIZone),    
        new ShooterCMD(m_shooter, m_intake, true, shooterConstants.shooterSpeed),
        new ParallelCommandGroup(
            new IntakeCMD(m_intake, true),
            new SequentialCommandGroup(
                //new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.AutoShortIZone),
                new AutoMoveCMD(exampleTrajectory).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
            )
        ),
        new ArmPIDCMD(m_arm_up, 30 + 90, false, ArmConstants.NormIZone),
        new ShooterCMD(m_shooter, m_intake, true, shooterConstants.fastShooterSpeed)
);

private Command Mobility =
    new SequentialCommandGroup(
        new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.AutoShortIZone),
        new AutoMoveCMD(exampleTrajectory).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
    );

private Command SideSubwoofer =
    new SequentialCommandGroup(
        new ArmPIDCMD(m_arm_up, ArmConstants.kSideSubwoofer + 90, false, ArmConstants.NormIZone),    
        new ShooterCMD(m_shooter, m_intake, true, shooterConstants.fastShooterSpeed),
        new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.AutoShortIZone)
    );

private Command NoAuto;

private Command SideMoveBlue = 
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ArmPIDCMD(m_arm_up, ArmConstants.kSideSubwoofer + 90, false, ArmConstants.NormIZone),    
                new ShooterCMD(m_shooter, m_intake, true, shooterConstants.fastShooterSpeed)),
            new SequentialCommandGroup(        
                new WaitCommand(12),
                new AutoMoveCMD(rotateBlue).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)))
        )
        //new GyroResetCMD(m_robotDrive, 60)
    );

private Command SideMoveRed = 
    new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ArmPIDCMD(m_arm_up, ArmConstants.kSideSubwoofer + 90, false, ArmConstants.NormIZone),    
                new ShooterCMD(m_shooter, m_intake, true, shooterConstants.fastShooterSpeed)),
            new SequentialCommandGroup(        
                new WaitCommand(12),
                new AutoMoveCMD(rotateRed).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)))
        )
        //new GyroResetCMD(m_robotDrive, -60)
    );

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    



    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(0.5, 0), new Translation2d(.7, 0)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(1.2, 0, new Rotation2d(1 * Math.PI / 180)),
    //     config);




    /*var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand() { 
        return new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    };

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());*/

    // An example trajectory to follow. All units in meters.
    /*Trajectory meterForward = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-0.25, 0), new Translation2d(-0.75, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-1, 0, new Rotation2d(0)),
        config);

    /*var thetaController2 = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController2.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        meterForward,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController2,
        m_robotDrive::setModuleStates,
        m_robotDrive);*/

    // Run path following command, then stop at the end.
    // return new SequentialCommandGroup(
    //     new ArmPIDCMD(m_arm_up, ArmConstants.kSpeaker + 90, false, ArmConstants.NormIZone),    
    //     new ShooterCMD(m_shooter, m_intake, true, shooterConstants.shooterSpeed),
    //     new ParallelCommandGroup(
    //         new IntakeCMD(m_intake, true),
    //         new SequentialCommandGroup(
    //             new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.AutoShortIZone),
    //             new AutoMoveCMD(exampleTrajectory).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
    //         )
    //     ),
    //     new ArmPIDCMD(m_arm_up, 30 + 90, false, ArmConstants.NormIZone),
    //     new ShooterCMD(m_shooter, m_intake, true, shooterConstants.fastShooterSpeed)
    // );
  }
}
//return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
//Other Auto's
/*
 //Main from Center. One Game Piece, Mobility, Pick up Game Piece.
 //Trajectory
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.5, 0), new Translation2d(.7, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.2, 0, new Rotation2d(1 * Math.PI / 180)),
        config);
//Sequential Command Group
    new ArmPIDCMD(m_arm_up, ArmConstants.kSpeaker + 90, false, ArmConstants.NormIZone),    
        new ShooterCMD(m_shooter, m_intake, true, shooterConstants.shooterSpeed),
        new ParallelCommandGroup(
            new IntakeCMD(m_intake, true),
            new SequentialCommandGroup(
                new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.AutoShortIZone),
                new AutoMoveCMD(exampleTrajectory).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
            )
        ),
        new ArmPIDCMD(m_arm_up, 30 + 90, false, ArmConstants.NormIZone),
        new ShooterCMD(m_shooter, m_intake, true, shooterConstants.fastShooterSpeed)
//Mobility, No Shoot, Arm to kSpeaker
 //Trajectory
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.5, 0), new Translation2d(.7, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.2, 0, new Rotation2d(1 * Math.PI / 180)),
        config);
//Sequential Command Group
    new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.AutoShortIZone),
    new AutoMoveCMD(exampleTrajectory).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
//One Game Piece, Mobility, Start on side BLUE ALLIANCE SOURCE SIDE
 //Trajectory
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.PI / 3)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, Math.sqrt(3))),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 2 * Math.sqrt(3), new Rotation2d(Math.PI / 3)),
        config);
//Sequential Command Group
        new ArmPIDCMD(m_arm_up, 15 + 90, false, ArmConstants.NormIZone),    
        new ShooterCMD(m_shooter, m_intake, true, shooterConstants.fastShooterSpeed),
        new ArmPIDCMD(m_arm_up, ArmConstants.kZeroPont + 90, false, ArmConstants.AutoShortIZone),
        //new AutoMoveCMD(exampleTrajectory).move().andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
 */