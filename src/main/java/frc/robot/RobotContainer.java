package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.Utils.PathPlanner.PathPlannerHelper;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.autos.*;


import org.photonvision.PhotonCamera;

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
    public static IntakeWheels intakeWheels;
    public static Wrist wrist;
    public static Arm arm;
    public static LEDSubsystem led;
  
    /* Controllers */
    //private final Joystick driver = new Joystick(0); // moved to OI for consistency

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

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
    
    private PhotonCamera photonFrontOVCamera =  null;
    private PhotonCamera photonBackOVCamera =  null;
    private PhotonCamera photonLifeCam = null;

   

    private  PoseEstimatorSubsystem poseEstimator = null;

    private ChaseTagCommand chaseTagCommand =  null;

    private ChaseNoteCommand chaseNoteCommand =  null;



   



    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();





    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        speakerShooter = new SpeakerShooter();
        intakeWheels = new IntakeWheels();
        led = new LEDSubsystem();
        oi = new OI();
         
         
        try {
            System.out.println("Ready to sleep for 5 seconds ...");
            Thread.sleep(5000); // try this
            photonFrontOVCamera = new PhotonCamera("Arducam_OV9281_USB_1");//JW's first camera
            photonBackOVCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");//AK's second camera

            //photonLifeCam = new PhotonCamera("USB_Camera");
            photonLifeCam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

            poseEstimator = new PoseEstimatorSubsystem(photonFrontOVCamera,  photonBackOVCamera, swerve);
            
            //  photonCamera may have race condition
            chaseTagCommand =  new ChaseTagCommand(photonFrontOVCamera, swerve, poseEstimator::getCurrentPose);

            chaseNoteCommand =  new ChaseNoteCommand(photonLifeCam, swerve, poseEstimator::getCurrentPose);

         
        }
        catch(Exception e) {

        }
        

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -oi.driverStick.getRawAxis(translationAxis), 
                () -> -oi.driverStick.getRawAxis(strafeAxis), 
                () -> -oi.driverStick.getRawAxis(rotationAxis), 
                //true,
                () -> oi.robotCentric.getAsBoolean(),
                () -> true
            )
        );

        intakeWheels.setDefaultCommand(new IntakeCommand(2));

        // Configure the button bindings
        configureButtonBindings();

       autoChooser.setDefaultOption("2 Piece Auto", new PathPlannerAuto("TwoPieceAuto"));
       autoChooser.addOption("Test auto", new PathPlannerAuto("TestAuto"));
       autoChooser.addOption("3 Piece Auto", new PathPlannerAuto("3Piece"));
       autoChooser.addOption("3 Piece Stage Auto", new PathPlannerAuto("3PieceStage"));
       autoChooser.addOption("4 Piece Auto", new PathPlannerAuto("4Piece"));
       autoChooser.addOption("4.5 Piece Auto", new PathPlannerAuto("4PieceLong"));
       autoChooser.addOption("5 Piece Auto", new PathPlannerAuto("5Piece"));
       //autoChooser.addOption("2 Piece Auto", new PathPlannerAuto("TwoPieceAuto"));
        //add more with autoChooser.addOption

        SmartDashboard.putData("Auto Route", autoChooser);

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

       // oi.turnLeft180Button.whileTrue(chaseTagCommand);

        oi.turnLeft180Button.whileTrue(chaseNoteCommand);
        oi.turnRight180Button.whileTrue(chaseTagCommand);
        oi.intakeDownButton.onTrue(new GrabDownCommand());
        oi.intakeUpButton.onTrue(new StoreCommand());

        //if(intakeWheels.noteSensor() > 1800){
            //new StoreCommand();
        //}
 
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

        return autoChooser.getSelected();
    }
}
