package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.autos.*;

import javax.swing.GroupLayout.SequentialGroup;



import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static OI oi;
    public static SpeakerShooter speakerShooter;
    public static Climb climb;
    //public static IntakeWheels intakeWheels;
    public static IntakeRollers intakeRollers;
    public static Wrist wrist;
    public static Arm arm;
    public static LEDSubsystem led;
    public static LEDSubsystemLeft ledLeft;
    //public static SwerveSubsystem swerveSubsystem;
  
    /* Controllers */
    //private final Joystick driver = new Joystick(0); // moved to OI for consistency

    /* Drive Controls */
    public static final int translationAxis = XboxController.Axis.kLeftY.value;
    public static final int strafeAxis = XboxController.Axis.kLeftX.value;
    public static final int rotationAxis = XboxController.Axis.kRightX.value;

    PathPlannerHelper pathPlannerHelper = PathPlannerHelper.getInstace();

    /* Driver Buttons */
    // moved to OI for consistency
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

   // private final SpeakerShooter s_SpeakerShooter = new SpeakerShooter();
   // private final IntakeWheels s_IntakeWheels = new IntakeWheels();

    // vision
   //private PhotonCamera photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    //private PhotonCamera photonCamera = new PhotonCamera("Arducam_OV9281_USB_1");
    // private PhotonCamera photonLifeCam = new PhotonCamera("USB_Camera");
    //private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, s_Swerve);
    // private ChaseTagCommand chaseTagCommand =  new ChaseTagCommand(photonCamera, s_Swerve, poseEstimator::getCurrentPose);
    
    public static PhotonCamera photonFrontOVCamera =  null;
    public static PhotonCamera photonBackOVCamera =  null;
    public static PhotonCamera photonLifeCam = null;

   

    public static  PoseEstimatorSubsystem poseEstimator = null;

    //private ChaseTagCommand2 chaseTagCommand2 =  null;
    //private ChaseTagCommand3 chaseTagCommand3 =  null;
    //private ChaseTagCommand4 chaseTagCommand4 =  null;
    private ChaseTagCommand3 chaseTagCommand3 =  null;

    private ChaseTagClimbCommand chaseTagClimbCommand =  null;

    //private ChaseNoteCommand chaseNoteCommand =  null;



   



    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();





    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        speakerShooter = new SpeakerShooter();
        //intakeWheels = new IntakeWheels();
        //swerveSubsystem = new SwerveSubsystem();
        arm = new Arm();
        climb = new Climb();
        intakeRollers = new IntakeRollers();
        wrist = new Wrist();
        led = new LEDSubsystem();
        //ledLeft = new LEDSubsystemLeft();
        oi = new OI();
         
        
        try {
            System.out.println("Ready to sleep for 5 seconds ...");
            Thread.sleep(5000); // try this
            //photonFrontOVCamera = new PhotonCamera("Arducam_OV9281_USB_1");//JW's first camera
           // photonFrontOVCamera = new PhotonCamera("Arducam_OV2311_USB_Camera");//JW's second different kind of camera

            photonBackOVCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");//AK's second camera
            
            //photonLifeCam = new PhotonCamera("USB_Camera");
            photonLifeCam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

            poseEstimator = new PoseEstimatorSubsystem(photonFrontOVCamera,  photonBackOVCamera, swerve);
            
            //  photonCamera may have race condition
            //chaseTagCommand2 =  new ChaseTagCommand2(photonBackOVCamera, swerve, poseEstimator::getCurrentPose);
            chaseTagCommand3 =  new ChaseTagCommand3(photonBackOVCamera, swerve, poseEstimator::getCurrentPose);
            //chaseTagCommand4 =  new ChaseTagCommand4(photonBackOVCamera, swerve, poseEstimator::getCurrentPose);

            chaseTagClimbCommand =  new ChaseTagClimbCommand(photonFrontOVCamera, swerve, poseEstimator::getCurrentPose);


            //chaseNoteCommand =  new ChaseNoteCommand(photonLifeCam, swerve, poseEstimator::getCurrentPose);
            
         
        }
        catch(Exception e) {
            System.out.println("error "+e);
        }
        

        NamedCommands.registerCommand("AutoWristSmartMotion(0)", new AutoWristSmartMotion(0));
        NamedCommands.registerCommand("AutoIntakeCommand", new AutoIntakeCommand());
        NamedCommands.registerCommand("StoreCommand", new StoreCommand(0.02));
        NamedCommands.registerCommand("SpeakerShooterCommand", new SpeakerShooterCommand());
        NamedCommands.registerCommand("AutoSpeakerShooterCommand", new AutoSpeakerShooterCommand());

        NamedCommands.registerCommand("DriveToNoteVisionTargetCommand", new DriveToNoteVisionTargetCommand(swerve , 2));
        NamedCommands.registerCommand("DriveToNoteVisionTargetCommand2", new DriveToNoteVisionTargetCommand2(swerve , 2));

        NamedCommands.registerCommand("ChaseSpeakerTagCommand", new ChaseSpeakerTagCommand(photonBackOVCamera, swerve, poseEstimator::getCurrentPose));
        NamedCommands.registerCommand("TurnToAbsoluteAngleCommand", new TurnToAbsoluteAngleCommand(swerve, 0.0, 2));


        /*
         
         NamedCommands.registerCommand("WristSmartMotion(0)", new WaitCommand(4));
        //NamedCommands.registerCommand("WristSmartMotion(0)", new TestLEDCommand(Color.kGreen,5.0));
        ///NamedCommands.registerCommand("AutoIntakeCommand", new TestLEDCommand(Color.kRed,5.0));
        NamedCommands.registerCommand("AutoIntakeCommand", new AutoIntakeCommand());
        NamedCommands.registerCommand("StoreCommand", new TestLEDCommand(Color.kBlue,5.0));
        //NamedCommands.registerCommand("SpeakerShooterCommand", new TestLEDCommand(Color.kGold,5.0));
        NamedCommands.registerCommand("SpeakerShooterCommand", new SpeakerShooterCommand());
        NamedCommands.registerCommand("ResetLEDCommand", new TestLEDCommand(Color.kPink));

         
         */



        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -oi.driverStick.getRawAxis(translationAxis), 
                () -> -oi.driverStick.getRawAxis(strafeAxis), 
                () -> -oi.driverStick.getRawAxis(rotationAxis), 
                //true,
                () -> oi.robotCentric.getAsBoolean(),
                () -> oi.slowSpeed.getAsBoolean()
                //() -> true
            )
        );

       // intakeWheels.setDefaultCommand(new IntakeCommand(2));
        //wrist.setDefaultCommand(new WristSmartMotion(0));

        intakeRollers.setDefaultCommand(new ManualIntakeRollersCommand());
        wrist.setDefaultCommand(new ManualWristCommand());

        // Configure the button bindings
        configureButtonBindings();

       //autoChooser.setDefaultOption("2 Piece Auto", new PathPlannerAuto("TwoPieceAuto"));
       //autoChooser.addOption("Test auto", new PathPlannerAuto("TestAuto"));
       //autoChooser.addOption("3 Piece Auto", new PathPlannerAuto("3Piece"));
       //autoChooser.addOption("3 Piece Stage Auto", new PathPlannerAuto("3PieceStage"));
       //autoChooser.addOption("4 Piece Auto", new PathPlannerAuto("4Piece"));
       //autoChooser.addOption("4.5 Piece Auto", new PathPlannerAuto("4PieceLong"));
       //autoChooser.addOption("5 Piece Auto", new PathPlannerAuto("5Piece"));
       autoChooser.setDefaultOption("Just Shoot Auto", new PathPlannerAuto("JustShootAuto"));//
       autoChooser.addOption("Center Two Piece Auto", new PathPlannerAuto("FrontToFrontAuto"));//
       autoChooser.addOption("Loading Side Two Piece Auto", new PathPlannerAuto("BottomTwoPieceAuto"));//
       autoChooser.addOption("Amp Side Two Piece Auto", new PathPlannerAuto("TopTwoPieceAuto"));//
       autoChooser.addOption("Amp Side Shoot and Leave Auto", new PathPlannerAuto("ShootAndLeaveTopAuto"));//
       autoChooser.addOption("Loading Side Shoot and Leave Auto", new PathPlannerAuto("ShootAndMoveBottomAuto"));//
       autoChooser.addOption("Center Start Amp Side 3 Piece Auto", new PathPlannerAuto("3PieceShortTopAuto"));//
       autoChooser.addOption("Center Start Load Side 3 Piece Auto", new PathPlannerAuto("3PieceShortLoadSideAuto"));//
       //autoChooser.addOption("Load Side 2 Piece Long Auto", new PathPlannerAuto("LoadSideToFourthNoteTwoPieceAuto"));
       autoChooser.addOption("Just Shoot Loading Side Auto", new PathPlannerAuto("JustShootLoadingSideAuto"));//
       autoChooser.addOption("Just Shoot Amp Side Auto", new PathPlannerAuto("JustShootAmpSideAuto"));//
       autoChooser.addOption("Do Nothing Auto", new PathPlannerAuto("DoNothingAuto"));//
       autoChooser.addOption("Center Start 4 Piece Amp Side Mid Note", new PathPlannerAuto("4PieceShortLoadSideAutoGoToMid"));
       autoChooser.addOption("Load Side 3 Note Defense Auto", new PathPlannerAuto("LoadSideShootThenDefenseAuto"));
       autoChooser.addOption("Load Side To Fourth Note Auto", new PathPlannerAuto("LoadSideToFourthNoteTwoPieceAuto"));//
       autoChooser.addOption("Load Side Far Start Fourth Note Two Piece", new PathPlannerAuto("SideToShootToFourthNoteAuto"));//
       autoChooser.addOption("Mid Start 4 Piece LoadMid Note (Load Side near)", new PathPlannerAuto("4PieceShortLoadSideAutoGoToLoadMidNote"));
       autoChooser.addOption("Load Start Sweep LoadMid and Mid Pick AmpMid Shoot", new PathPlannerAuto("Sweep2PickCenterAmpAuto"));

       //autoChooser.addOption("2 Piece Auto", new PathPlannerAuto("TwoPieceAuto"));
        //add more with autoChooser.addOption

        SmartDashboard.putData("Auto Route", autoChooser);

        CameraServer.startAutomaticCapture(0);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
       // oi.zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //oi.turnLeft180Button.onTrue(new TurnToAngleCommand(s_Swerve, 180, 3));

        /* 
        oi.turnLeft90Button.onTrue(new AutoDropNoteAmpSeqCommand(photonBackOVCamera, swerve, poseEstimator::getCurrentPose));
        oi.turnRight90Button.onTrue(testAutoDriveToPose());

        oi.turnLeft180Button.onTrue(chaseNoteCommand);
        oi.turnRight180Button.onTrue(chaseTagCommand);
        */

       // oi.turnLeft90Button.onTrue(TestSupperStructureAuto());


        //oi.turnLeft180Button.whileTrue(chaseNoteCommand);
       // oi.turnRight180Button.whileTrue(chaseTagCommand2);
       oi.turnRight180Button.whileTrue(chaseTagCommand3);
        //oi.turnRight180Button.whileTrue(chaseTagCommand4);
       // oi.intakeDownButton.onTrue(new GrabDownCommand());
        //oi.intakeUpButton.onTrue(new StoreCommand());
        oi.climbPosButton.onTrue(new ClimbPositionCommand());
        oi.climbUpButton.whileTrue(new ClimbUpCommand());

        oi.hulkButton.whileTrue(new InstantCommand(() -> swerve.configToX()));

        oi.resetSwerveButton.onTrue(new InstantCommand(
            () -> swerve.zeroGyro()
        ));

        oi.resetWristButton.onTrue(new InstantCommand(
            () -> wrist.resetEncoders()
        ));

        SmartDashboard.putNumber("DropNoteAmpCommandWristPosition",-9.5);// as default, can be modified in Shuffleboard
        SmartDashboard.putNumber("DropNoteAmpCommandArmPositionMode3",49);

        oi.ampArmButton.onTrue(new ArmAmpCommand().andThen(new StoreCommand(2)));
        //oi.afterAmpStoreButton.onTrue(new StoreCommand(1));

        oi.ampButton.onTrue(new AmpShooterCommand());

       
        //oi.intakeUpButton.onTrue(new InstantCommand(intakeRollers::stop));
        oi.intakeUpButton.onTrue(new StoreCommand(0.02));
        oi.speakerShooterButton.onTrue(new SpeakerShooterCommand());
        //oi.intakeDownButton.onTrue(intakeRollers.intakeGamepieceCommand().andThen(new StoreCommand())).onFalse(new InstantCommand(intakeRollers::stop)); //may make the onFalse a store command
        //oi.intakeDownButton.onTrue(intakeRollers.intakeGamepieceCommand().andThen(new WristSmartMotion(0))).onFalse(new InstantCommand(intakeRollers::stop)); //may make the onFalse a store command
      // oi.intakeDownButton.onTrue(new WristSmartMotion(0)); //may make the onFalse a store command
        //oi.intakeDownButton.onTrue(intakeRollers.intakeGamepieceCommand());
        //oi.intakeDownButton.onTrue(new ParallelCommandGroup(intakeRollers.intakeGamepieceCommand(), new WristSmartMotion(0)));
        //oi.ampButton.onTrue((new DropNoteAmpCommand()).andThen(new StoreCommand(1.5)));
        //oi.ampButton.onTrue(new ParallelCommandGroup())
        //oi.resetWristButton.onTrue(wrist.resetEncoders());
        //oi.resetWristButton.onTrue(new ResetWristCommand());
        oi.intakeDownButton.onTrue((new ParallelDeadlineGroup(intakeRollers.intakeGamepieceCommand(), new WristSmartMotion(0))).andThen(new ParallelDeadlineGroup(new StoreCommand(0.02), intakeRollers.centerNoteCommand())));
        //oi.intakeDownButton.onTrue((new ParallelDeadlineGroup(intakeRollers.intakeGamepieceCommand(), new WristSmartMotion(0))).andThen(new StoreCommand(0.02)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(swerve); // not work anymore with pathplaner enabled
        //return new PathPlannerAuto("TestAuto");
        //return new PathPlannerAuto("TwoPiece");
        //return new PathPlannerAuto("TwoPieceAuto");// works, but robot not turning as GUI shown
        // return new PathPlannerAuto("3Piece");
        //return new PathPlannerAuto("4Piece");
        //return new PathPlannerAuto("5Piece");
        //return new PathPlannerAuto("4PieceLong");

        //return Speaker2NoteAuto();

        //return TestSuperStructureAuto();

        //return Speaker2NoteAutoWithoutPathPlanner();

        //return BlueTwoPieceCenterToLeftAuto();


        return autoChooser.getSelected();
    }


    public Command LoadSideToFourthNoteTwoPieceAuto(){
        String autoName = "LoadSideToFourthNoteTwoPieceAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

     public Command Sweep2PickCenterAmpAuto(){
        String autoName = "Sweep2PickCenterAmpAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command DoNothingAuto(){
        String autoName = "DoNothingAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

     public Command SideToShootToFourthNoteAuto(){
        String autoName = "SideToShootToFourthNoteAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

     public Command LoadSideShootThenDefenseAuto(){
        String autoName = "LoadSideShootThenDefenseAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

     public Command Speaker2NoteAuto(){
        String autoName = "TwoPieceAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command ThreePieceShortTopAuto(){
        String autoName = "3PieceShortTopAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

     public Command ThreePieceShortLoadSideAuto(){
        String autoName = "3PieceShortLoadSideAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }


    public Command JustShootAuto(){
        String autoName = "JustShootAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command JustShootLoadingSideAuto(){
        String autoName = "JustShootLoadingSideAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }
    
    public Command JustShootAmpSideAuto(){
        String autoName = "JustShootAmpSideAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command FrontToFrontAuto(){
        String autoName = "FrontToFrontAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command BottomTwoPieceAuto(){
        String autoName = "BottomTwoPieceAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command TopTwoPieceAuto(){
        String autoName = "TopTwoPieceAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command ShootAndLeaveTopAuto(){
        String autoName = "ShootAndLeaveTopAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    public Command ShootAndMoveBottomAuto(){
        String autoName = "ShootAndMoveBottomAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }

    
    public Command BlueTwoPieceCenterToLeftAuto(){
        String autoName = "BlueTwoPieceCenterToLeftAuto";
        Pose2d pose =  PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        pose);
        
        return new PathPlannerAuto(autoName);
    }




    // working but not back to the exact origin
    public Command Speaker2NoteAutoWithoutPathPlanner(){
        // use our homegrown AutoDriveToTargetPoseCommand with proper timeout when the location/pose is known and not far away

        // Blue 2 Piece Center to Center Auto
        Pose2d robotPose2d = new Pose2d(1.44,5.55,  Rotation2d.fromDegrees(0+ swerve.getYaw().getDegrees())   );
        
        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        robotPose2d);
       
        //
        //Pose2d endPose2dCenter = new Pose2d(2.58,5.55,  Rotation2d.fromDegrees(0+ swerve.getYaw().getDegrees())   );
        //Pose2d endPose2dRight = new Pose2d(2.58,4.1,  Rotation2d.fromDegrees(-51.7+ swerve.getYaw().getDegrees())   );
        Pose2d endPose2dLeft = new Pose2d(2.58,7.0,  Rotation2d.fromDegrees(51.7+ swerve.getYaw().getDegrees())   );

        SequentialCommandGroup roundtripCmd = new  SequentialCommandGroup(
                new AutoDriveToTargetPoseCommand(endPose2dLeft ,swerve, swerve::getPose, 2.0),
                new TestLEDCommand(Color.kBlue,2.0),
                new AutoDriveToTargetPoseCommand(robotPose2d,swerve, swerve::getPose, 2.0),
                new TestLEDCommand(Color.kGold,2.0)
        );

       return roundtripCmd;
       // return new AutoDriveToTargetPoseCommand(endPose2dLeft ,swerve, swerve::getPose, 2.0);
    }




    public Command testAutoDriveToPose(){
       
        
        Pose2d initialRobotPos2d = poseEstimator.getCurrentPose();
        swerve.swerveOdometry.resetPosition(swerve.getYaw(), swerve.getModulePositions(), 
        initialRobotPos2d);

        //Pose2d targetPose2d = new Pose2d(1,1,swerve.getYaw());
        Pose2d targetPose2d = new Pose2d(1,0,  Rotation2d.fromDegrees(0+ swerve.getYaw().getDegrees())   );
        Transform2d robotToGoal = new Transform2d(0.5,0.5, initialRobotPos2d.getRotation()) ; // has issue with only y change, not moving

        System.out.println("current pose = "+initialRobotPos2d);
        //System.out.println("target pose = "+targetPose2d);
        
        return new AutoDriveToTargetPoseCommand(robotToGoal ,swerve, swerve::getPose);
        //return new AutoDriveToTargetPoseCommand(targetPose2d,swerve, swerve::getPose); // working well
        //return new AutoDriveToTargetPoseCommand(targetPose2d,swerve,poseEstimator::getCurrentPose);
    }

}
