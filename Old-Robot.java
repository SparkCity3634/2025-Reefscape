// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI.Port;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import frc.robot.RobotContainer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import java.lang.Math;
import javax.xml.xpath.XPathExpression;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private RobotContainer m_robotContainer1;
  private Command m_autonomousCommand;

  //private RobotContainer m_robotContainer;
    //These are the motor bindings needed for our drivetrain, used in the SwerveModule
  
  private final CANSparkMax BackRightDrive = new CANSparkMax(41, MotorType.kBrushless);
  private final CANSparkMax BackRightTurn = new CANSparkMax(42, MotorType.kBrushless);
  private final CANSparkMax FrontRightDrive = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax FrontRightTurn = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax FrontLeftDrive = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax FrontLeftTurn = new CANSparkMax(22, MotorType.kBrushless);
  private final CANSparkMax BackLeftDrive = new CANSparkMax(31, MotorType.kBrushless);
  private final CANSparkMax BackLeftTurn = new CANSparkMax(32, MotorType.kBrushless);
  
  private RelativeEncoder m_BackLeftDriveEncoder;
  private RelativeEncoder m_BackRightDriveEncoder;
  private RelativeEncoder m_FrontRightDriveEncoder;
  private RelativeEncoder m_FrontLeftDriveEncoder;

  private RelativeEncoder m_BackLeftTurnEncoder;
  private RelativeEncoder m_BackRightTurnEncoder;
  private RelativeEncoder m_FrontRightTurnEncoder;
  private RelativeEncoder m_FrontLeftTurnEncoder;

  private RelativeEncoder m_ShooterAngleEncoder;
  private RelativeEncoder m_ShooterLeftEncoder;
  private RelativeEncoder m_ShooterRightEncoder;


  //Bind CANcoders for Absolute Turn Encoding
  private static final String canBusName = "rio";
  private final CANcoder m_FrontRightTurnCancoder = new CANcoder(10, canBusName);
  private final CANcoder m_FrontLeftTurnCancoder = new CANcoder(20, canBusName);
  private final CANcoder m_BackLeftTurnCancoder = new CANcoder(30, canBusName);
  private final CANcoder m_BackRightTurnCancoder = new CANcoder(40, canBusName);
  private final DutyCycleOut fwdOut = new DutyCycleOut(0);

  //Bind Module controllers
  private final CANSparkMax intakeTop = new CANSparkMax(57, MotorType.kBrushless);
  private final CANSparkMax intakeBot = new CANSparkMax(56, MotorType.kBrushless);
  private final CANSparkMax hangLeft = new CANSparkMax(61, MotorType.kBrushless);
  private final CANSparkMax hangRight = new CANSparkMax(60, MotorType.kBrushless);
  private final CANSparkMax shooterLeft = new CANSparkMax(53, MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(55, MotorType.kBrushless);
  private final CANSparkMax shooterLift = new CANSparkMax(51, MotorType.kBrushless);

  
  XboxController o_controller = new XboxController(1);  
  XboxController m_controller = new XboxController(0);  
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  Timer m_Timer = new Timer();
//private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  //create SparkMaxPIDControllers to fix turn motors
  private SparkMaxPIDController m_BackRightTurnPID;
  private SparkMaxPIDController m_FrontRightTurnPID;
  private SparkMaxPIDController m_FrontLeftTurnPID;
  private SparkMaxPIDController m_BackLeftTurnPID;
  
  private SparkMaxPIDController m_BackLeftDrivePID;
  private SparkMaxPIDController m_BackRightDrivePID;
  private SparkMaxPIDController m_FrontRightDrivePID;
  private SparkMaxPIDController m_FrontLeftDrivePID;

  private SparkMaxPIDController m_ShooterAnglePID;
  private SparkMaxPIDController m_ShooterLeftPID;
  private SparkMaxPIDController m_ShooterRightPID;

  public double ktP, ktI, ktD, ktIz, ktFF, ktMaxOutput, ktMinOutput, maxRPM, setTurn;
  public double kdP, kdI, kdD, kdIz, kdFF, kdMaxOutput, kdMinOutput;
  public double anglekP, anglekI, anglekD, anglekIz, anglekFF, anglekMinInput, anglekMaxOutput;
  public double ksP, ksI, ksD, ksIz, ksFF, ksMaxOutput, ksMinOutput;
  public double xAxis, yAxis, kYawRate, maxVel, maxYaw, o_lyAxis, o_ryAxis;
  public double k_posConv = .048; // linear meters traveled at wheel, per motor rotation
  public double k_velConv = .0008106; // linear meters per second (m/s) speed at wheel, convert from motor RPM
  public double k_turnConv = 16.8; //wheel degrees per motor rotation from steering relative encoder
  public double rotations = 0;
  public double k_ShooterVelConv = .005319; // linear meters per second (m/s) speed at shooter, convert from motor RPM
  public double maxShootVelocity;

  public double traplocation = -22; 

//Create Swerve Kinematics modules
Translation2d m_BackLeftLocation = new Translation2d(-0.25 ,0.250);
Translation2d m_BackRightLocation = new Translation2d(-0.25 ,-.250);
Translation2d m_FrontLeftLocation = new Translation2d(.250,.250);
Translation2d m_FrontRightLocation = new Translation2d(.250,-.250);

// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

public final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_BackLeftLocation, m_BackRightLocation, m_FrontLeftLocation, m_FrontRightLocation);
 
private static final String kRedAuto = "Red";
private static final String kBlueAuto = "Blue";
private String m_autoSelected;
private final SendableChooser<String> m_chooser = new SendableChooser<>();

ChassisSpeeds fieldspeeds = new ChassisSpeeds(0,0,0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //m_robotContainer1 = new RobotContainer();
    
    //set up our USB Camera 

    m_chooser.setDefaultOption("Red Auto", kRedAuto);
    m_chooser.addOption("Blue Auto", kBlueAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    camera1.setResolution(320, 240);
    camera2.setResolution(320, 240);
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    
    BackLeftDrive.restoreFactoryDefaults();
    BackRightDrive.restoreFactoryDefaults();
    FrontLeftDrive.restoreFactoryDefaults();
    FrontRightDrive.restoreFactoryDefaults();

    shooterLift.restoreFactoryDefaults();

    
    /* Configure CANcoder */
    //var toApply = new CANcoderConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    /*m_FrontRightTurnCancoder.getConfigurator().apply(toApply);
    m_FrontLeftTurnCancoder.getConfigurator().apply(toApply);
    m_BackLeftTurnCancoder.getConfigurator().apply(toApply);
    m_BackRightTurnCancoder.getConfigurator().apply(toApply);*/



    /* Speed up signals to an appropriate rate */
    m_FrontRightTurnCancoder.getPosition().setUpdateFrequency(100);
    m_FrontRightTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_FrontLeftTurnCancoder.getPosition().setUpdateFrequency(100);
    m_FrontLeftTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_BackLeftTurnCancoder.getPosition().setUpdateFrequency(100);
    m_BackLeftTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_BackRightTurnCancoder.getPosition().setUpdateFrequency(100);
    m_BackRightTurnCancoder.getVelocity().setUpdateFrequency(100);

    //Enable Idle Coast on Steer Motors
    FrontLeftTurn.setIdleMode(IdleMode.kCoast);
    FrontRightTurn.setIdleMode(IdleMode.kCoast);
    BackLeftTurn.setIdleMode(IdleMode.kCoast);
    BackRightTurn.setIdleMode(IdleMode.kCoast);
    BackLeftDrive.setIdleMode(IdleMode.kCoast);
    BackRightDrive.setIdleMode(IdleMode.kCoast);
    FrontRightDrive.setIdleMode(IdleMode.kCoast);
    FrontLeftDrive.setIdleMode(IdleMode.kCoast);
    FrontLeftTurn.setSmartCurrentLimit(40);
    FrontRightTurn.setSmartCurrentLimit(40);
    BackLeftTurn.setSmartCurrentLimit(40);
    BackRightTurn.setSmartCurrentLimit(40);

    hangLeft.setSmartCurrentLimit(30);
    hangRight.setSmartCurrentLimit(30);


    //declare turn sensors (NEO Relative encoders)
    m_FrontRightTurnEncoder = FrontRightTurn.getEncoder();
    m_BackRightTurnEncoder = BackRightTurn.getEncoder();
    m_FrontLeftTurnEncoder = FrontLeftTurn.getEncoder();
    m_BackLeftTurnEncoder = BackLeftTurn.getEncoder();
    
    //set conversion factor for 0 to 360 degrees
    m_BackRightTurnEncoder.setPositionConversionFactor(k_turnConv);
    m_BackRightTurnEncoder.setVelocityConversionFactor(k_turnConv);
    m_FrontRightTurnEncoder.setPositionConversionFactor(k_turnConv);
    m_FrontRightTurnEncoder.setVelocityConversionFactor(k_turnConv);
    m_FrontLeftTurnEncoder.setPositionConversionFactor(k_turnConv);
    m_FrontLeftTurnEncoder.setVelocityConversionFactor(k_turnConv);
    m_BackLeftTurnEncoder.setPositionConversionFactor(k_turnConv);
    m_BackLeftTurnEncoder.setVelocityConversionFactor(k_turnConv);
    
    m_BackRightTurnPID = BackRightTurn.getPIDController();
    m_FrontRightTurnPID = FrontRightTurn.getPIDController();
    m_FrontLeftTurnPID = FrontLeftTurn.getPIDController();
    m_BackLeftTurnPID = BackLeftTurn.getPIDController();

    m_BackLeftDrivePID = BackLeftDrive.getPIDController();
    m_BackRightDrivePID = BackRightDrive.getPIDController();
    m_FrontLeftDrivePID = FrontLeftDrive.getPIDController();
    m_FrontRightDrivePID = FrontRightDrive.getPIDController();

    //Create shooter encoders and PIDcontrollers
    m_ShooterAngleEncoder = shooterLift.getEncoder();
    m_ShooterAnglePID = shooterLift.getPIDController();
    m_ShooterLeftEncoder = shooterLeft.getEncoder();
    m_ShooterLeftPID = shooterLeft.getPIDController();
    m_ShooterRightEncoder = shooterRight.getEncoder();
    m_ShooterRightPID = shooterRight.getPIDController();
   
    // Encoder object created to display position values
    // set conversion from motor RPM to wheel linear meters and meters/second
    // with SDS Swerve gear ratio of 6.75 from motor to wheel, wheel diameter of 4" = .314m conv factor = ??
    // max NEO speed of 5700 rpm should equal ~15.1 ft/sec = ??m/s
    m_BackLeftDriveEncoder = BackLeftDrive.getEncoder();
    m_BackLeftDriveEncoder.setPositionConversionFactor(k_posConv);
    m_BackLeftDriveEncoder.setVelocityConversionFactor(k_velConv);
    m_BackLeftDriveEncoder.setPosition(0);
    m_BackRightDriveEncoder = BackRightDrive.getEncoder();
    m_BackRightDriveEncoder.setPositionConversionFactor(k_posConv);
    m_BackRightDriveEncoder.setVelocityConversionFactor(k_velConv);
    m_BackRightDriveEncoder.setPosition(0);
    m_FrontLeftDriveEncoder = FrontLeftDrive.getEncoder();
    m_FrontLeftDriveEncoder.setPositionConversionFactor(k_posConv);
    m_FrontLeftDriveEncoder.setVelocityConversionFactor(k_velConv);
    m_FrontLeftDriveEncoder.setPosition(0);
    m_FrontRightDriveEncoder = FrontRightDrive.getEncoder();
    m_FrontRightDriveEncoder.setPositionConversionFactor(k_posConv);
    m_FrontRightDriveEncoder.setVelocityConversionFactor(k_velConv);
    m_FrontRightDriveEncoder.setPosition(0);

    m_ShooterAngleEncoder.setPosition(0);
    m_ShooterLeftEncoder.setVelocityConversionFactor(k_ShooterVelConv);
    m_ShooterRightEncoder.setVelocityConversionFactor(k_ShooterVelConv);


    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the SparkMaxAnalogSensor object. 
     */
    m_BackRightTurnPID.setFeedbackDevice(m_BackRightTurnEncoder);
    m_BackRightTurnPID.setPositionPIDWrappingEnabled(true);
    m_BackRightTurnPID.setPositionPIDWrappingMinInput(0);
    m_BackRightTurnPID.setPositionPIDWrappingMaxInput(360);

    m_FrontRightTurnPID.setFeedbackDevice(m_FrontRightTurnEncoder);
    m_FrontRightTurnPID.setPositionPIDWrappingEnabled(true);
    m_FrontRightTurnPID.setPositionPIDWrappingMinInput(0);
    m_FrontRightTurnPID.setPositionPIDWrappingMaxInput(360);

    m_FrontLeftTurnPID.setFeedbackDevice(m_FrontLeftTurnEncoder);
    m_FrontLeftTurnPID.setPositionPIDWrappingEnabled(true);
    m_FrontLeftTurnPID.setPositionPIDWrappingMinInput(0);
    m_FrontLeftTurnPID.setPositionPIDWrappingMaxInput(360);

    m_BackLeftTurnPID.setFeedbackDevice(m_BackLeftTurnEncoder);
    m_BackLeftTurnPID.setPositionPIDWrappingEnabled(true);
    m_BackLeftTurnPID.setPositionPIDWrappingMinInput(0);
    m_BackLeftTurnPID.setPositionPIDWrappingMaxInput(360);

    m_BackLeftDrivePID.setFeedbackDevice(m_BackLeftDriveEncoder);
    m_BackRightDrivePID.setFeedbackDevice(m_BackRightDriveEncoder);
    m_FrontRightDrivePID.setFeedbackDevice(m_FrontRightDriveEncoder);
    m_FrontLeftDrivePID.setFeedbackDevice(m_FrontLeftDriveEncoder);
    FrontLeftDrive.setInverted(false);
    FrontRightDrive.setInverted(false);
    BackLeftDrive.setInverted(false);
    BackRightDrive.setInverted(false);

    BackRightTurn.setInverted(true);
    FrontLeftTurn.setInverted(true);
    FrontRightTurn.setInverted(true);
    BackLeftTurn.setInverted(true);

    m_ShooterLeftPID.setFeedbackDevice(m_ShooterLeftEncoder);
    m_ShooterRightPID.setFeedbackDevice(m_ShooterRightEncoder);
    shooterRight.setInverted(true);
    
    m_ShooterAnglePID.setFeedbackDevice(m_ShooterAngleEncoder);

    //Initialize RelativeEncoders to CANCoder Absolute Value
    m_FrontRightTurnEncoder.setPosition(m_FrontRightTurnCancoder.getAbsolutePosition().getValue() * 360);//might want to add the .waitForUpdate() method to reduce latency?
    m_FrontLeftTurnEncoder.setPosition(m_FrontLeftTurnCancoder.getAbsolutePosition().getValue() * 360);
    m_BackLeftTurnEncoder.setPosition(m_BackLeftTurnCancoder.getAbsolutePosition().getValue() * 360);
    m_BackRightTurnEncoder.setPosition(m_BackRightTurnCancoder.getAbsolutePosition().getValue() * 360);

    // Drive Position PID coefficients (used for Autonomous Control)
    kdP = 0.4; 
    kdI = 0.00000;
    kdD = 0.05; 
    kdIz = 0; 
    kdFF = 0.01; 
    kdMaxOutput = 1; 
    kdMinOutput = -1;
    maxVel = 4; // m/s linear velocity of drive wheel
    maxYaw = 2*Math.PI;   // max rad/s for chassis rotation rate
    
    // Turn PID coefficients
    ktP = 0.07; //baseline was .07
    ktI = 0.0000;
    ktD = 0.001; //baseline was .1
    ktIz = 0; 
    ktFF = 0.003; //baseline was 0
    ktMaxOutput = .9; 
    ktMinOutput = -.9;

    // Angle Adjustor PID
    anglekP = 0.03;
    anglekI = 0.0000;
    anglekD = 0.001;
    anglekIz = 0; 
    anglekFF = 0.006;
    anglekMinInput = -1.0;
    anglekMaxOutput = 1.0;
    
    //Shooter PID
    ksP= 0.0005;
    ksI = 0.0000;
    ksD = 0.05;
    ksIz = 0;
    ksFF = 0.001;
    ksMaxOutput = 1;
    ksMinOutput = -1;
    maxShootVelocity = 175; //shooter linear velocity at wheel in m/s


    // set PID coefficients

    m_ShooterAnglePID.setP(anglekP);
    m_ShooterAnglePID.setI(anglekI);
    m_ShooterAnglePID.setD(anglekD);
    m_ShooterAnglePID.setIZone(anglekIz);
    m_ShooterAnglePID.setFF(anglekFF);
    m_ShooterAnglePID.setOutputRange(anglekMinInput, anglekMaxOutput);

    m_ShooterLeftPID.setP(ksP);
    m_ShooterLeftPID.setI(ksI);
    m_ShooterLeftPID.setD(ksD);
    m_ShooterLeftPID.setIZone(ksIz);
    m_ShooterLeftPID.setFF(ksFF);
    m_ShooterLeftPID.setOutputRange(ksMinOutput, ksMaxOutput);

    m_ShooterRightPID.setP(anglekP);
    m_ShooterRightPID.setI(anglekI);
    m_ShooterRightPID.setD(anglekD);
    m_ShooterRightPID.setIZone(anglekIz);
    m_ShooterRightPID.setFF(anglekFF);
    m_ShooterRightPID.setOutputRange(anglekMinInput, anglekMaxOutput);


    m_BackRightTurnPID.setP(ktP);
    m_BackRightTurnPID.setI(ktI);
    m_BackRightTurnPID.setD(ktD);
    m_BackRightTurnPID.setIZone(ktIz);
    m_BackRightTurnPID.setFF(ktFF);
    m_BackRightTurnPID.setOutputRange(ktMinOutput, ktMaxOutput);

    m_FrontRightTurnPID.setP(ktP);
    m_FrontRightTurnPID.setI(ktI);
    m_FrontRightTurnPID.setD(ktD);
    m_FrontRightTurnPID.setIZone(ktIz);
    m_FrontRightTurnPID.setFF(ktFF);
    m_FrontRightTurnPID.setOutputRange(ktMinOutput, ktMaxOutput);

    m_FrontLeftTurnPID.setP(ktP);
    m_FrontLeftTurnPID.setI(ktI);
    m_FrontLeftTurnPID.setD(ktD);
    m_FrontLeftTurnPID.setIZone(ktIz);
    m_FrontLeftTurnPID.setFF(ktFF);
    m_FrontLeftTurnPID.setOutputRange(ktMinOutput, ktMaxOutput);

    m_BackLeftTurnPID.setP(ktP);
    m_BackLeftTurnPID.setI(ktI);
    m_BackLeftTurnPID.setD(ktD);
    m_BackLeftTurnPID.setIZone(ktIz);
    m_BackLeftTurnPID.setFF(ktFF);
    m_BackLeftTurnPID.setOutputRange(ktMinOutput, ktMaxOutput);

     m_BackLeftDrivePID.setP(kdP);
     m_BackLeftDrivePID.setI(kdI);
     m_BackLeftDrivePID.setD(kdD);
     m_BackLeftDrivePID.setIZone(kdIz);
     m_BackLeftDrivePID.setFF(kdFF);
     m_BackLeftDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

     m_BackRightDrivePID.setP(kdP);
     m_BackRightDrivePID.setI(kdI);
     m_BackRightDrivePID.setD(kdD);
     m_BackRightDrivePID.setIZone(kdIz);
     m_BackRightDrivePID.setFF(kdFF);
     m_BackRightDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

     m_FrontLeftDrivePID.setP(kdP);
     m_FrontLeftDrivePID.setI(kdI);
     m_FrontLeftDrivePID.setD(kdD);
     m_FrontLeftDrivePID.setIZone(kdIz);
     m_FrontLeftDrivePID.setFF(kdFF);
     m_FrontLeftDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

     m_FrontRightDrivePID.setP(kdP);
     m_FrontRightDrivePID.setI(kdI);
     m_FrontRightDrivePID.setD(kdD);
     m_FrontRightDrivePID.setIZone(kdIz);
     m_FrontRightDrivePID.setFF(kdFF);
     m_FrontRightDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

    //Set Initial Swerve States
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);
    SwerveModuleState BackLeftSwerve = moduleStates[0];
    SwerveModuleState BackRightSwerve = moduleStates[1];
    SwerveModuleState FrontLeftSwerve = moduleStates[2];
    SwerveModuleState FrontRightSwerve = moduleStates[3];
    SwerveModuleState BackLeftOptimized = SwerveModuleState.optimize(BackLeftSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_BackLeftTurnEncoder.getPosition(),0 , 360)));
    SwerveModuleState BackRightOptimized = SwerveModuleState.optimize(BackRightSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_BackRightTurnEncoder.getPosition(),0 , 360)));
    SwerveModuleState FrontLeftOptimized = SwerveModuleState.optimize(FrontLeftSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_FrontLeftTurnEncoder.getPosition(),0 , 360)));
    SwerveModuleState FrontRightOptimized = SwerveModuleState.optimize(FrontRightSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_FrontRightTurnEncoder.getPosition(),0 , 360)));
   
    m_gyro.reset();
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kdP);
    //SmartDashboard.putNumber("Get P", m_BackLeftDrivePID.getP());
    SmartDashboard.putNumber("I Gain", kdI);
    //SmartDashboard.putNumber("Get I", m_BackLeftDrivePID.getI());
    SmartDashboard.putNumber("D Gain", kdD);    
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double tx = LimelightHelpers.getTX("");
    System.out.println("The value of tx is: " + tx);
    /**Cancoder Abs Position updates the CANSparkMax Relative Encoders to reduce drift
       * getPosition automatically calls refresh(), no need to manually refresh.
       * 
       * StatusSignalValues also have the toString method implemented, to provide
       * a useful print of the signal.
       */
      /*m_FrontRightTurnEncoder.setPosition(m_FrontRightTurnCancoder.getPosition().getValue() * 360);//might want to add the .waitForUpdate() method to reduce latency?
      m_FrontLeftTurnEncoder.setPosition(m_FrontLeftTurnCancoder.getPosition().getValue() * 360);
      m_BackLeftTurnEncoder.setPosition(m_BackLeftTurnCancoder.getPosition().getValue() * 360);
      m_BackRightTurnEncoder.setPosition(m_BackRightTurnCancoder.getPosition().getValue() * 360);*/


  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    /*m_autonomousCommand = m_robotContainer1.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    //insert Auto initiatlization here, set Drive control to position
    */
    m_BackRightTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
    m_FrontRightTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
    m_FrontLeftTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
    m_BackLeftTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);  
      
    m_BackLeftDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_BackRightDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_FrontLeftDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_FrontRightDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_Timer.restart();

  }
  /** This function is called periodically during autonomous. */
 @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kRedAuto:
      if(m_Timer.hasElapsed(2) != true) {
        //shooterLeft.set(-0.7);
        //shooterRight.set(0.4);
        m_ShooterLeftPID.setReference(-.6 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
        m_ShooterRightPID.setReference(-.3 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
      }
      else if(m_Timer.hasElapsed(5) != true) {
        intakeTop.set(1);
      }
      else if(m_Timer.hasElapsed(6) != true) {
        //shooterLeft.set(0.0);  //shooter stops
        //shooterRight.set(0.0);
        m_ShooterLeftPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        m_ShooterRightPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        intakeBot.set(0);
        intakeTop.set(0);    
      }
      else if(m_Timer.hasElapsed(10) != true) {
        m_BackRightTurnPID.setReference(-25, CANSparkMax.ControlType.kPosition);
        m_FrontRightTurnPID.setReference(-25, CANSparkMax.ControlType.kPosition);
        m_FrontLeftTurnPID.setReference(-25, CANSparkMax.ControlType.kPosition);
        m_BackLeftTurnPID.setReference(-25, CANSparkMax.ControlType.kPosition);  
      
        m_BackLeftDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
        m_BackRightDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
        m_FrontLeftDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
        m_FrontRightDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
      }
      else{
          m_BackRightTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
          m_FrontRightTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
          m_FrontLeftTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
          m_BackLeftTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);  
      
          m_BackLeftDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
          m_BackRightDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
          m_FrontLeftDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
          m_FrontRightDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
      }
      break;
      case kBlueAuto:
      if(m_Timer.hasElapsed(2) != true) {
        m_ShooterLeftPID.setReference(-.6 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
        m_ShooterRightPID.setReference(-.3 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
      }
      else if(m_Timer.hasElapsed(5) != true) {
        intakeTop.set(1);
      }
      else if(m_Timer.hasElapsed(6) != true) {
        m_ShooterLeftPID.setReference(0.0 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
        m_ShooterRightPID.setReference(0.0 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
        intakeBot.set(0);
        intakeTop.set(0);    
      }
      else if(m_Timer.hasElapsed(10) != true) {
        m_BackRightTurnPID.setReference(25, CANSparkMax.ControlType.kPosition);
        m_FrontRightTurnPID.setReference(25, CANSparkMax.ControlType.kPosition);
        m_FrontLeftTurnPID.setReference(25, CANSparkMax.ControlType.kPosition);
        m_BackLeftTurnPID.setReference(25, CANSparkMax.ControlType.kPosition);  
      
        m_BackLeftDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
        m_BackRightDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
        m_FrontLeftDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
        m_FrontRightDrivePID.setReference(-1, CANSparkMax.ControlType.kVelocity);
      }
      else{
          m_BackRightTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
          m_FrontRightTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
          m_FrontLeftTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);
          m_BackLeftTurnPID.setReference(0, CANSparkMax.ControlType.kPosition);  
      
          m_BackLeftDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
          m_BackRightDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
          m_FrontLeftDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
          m_FrontRightDrivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
      }
      break;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_gyro.reset();
    //Reinitialize Velocity drive mode for teleop with correct PIDs
    // Drive Velocity PID coefficients and max rates (need to retune for Velocity Control)
    kdP = 0.4; 
    kdI = 0.00000;
    kdD = 0.05; 
    kdIz = 0; 
    kdFF = 0.01; 
    kdMaxOutput = 1; 
    kdMinOutput = -1;
    maxVel = 4; // m/s linear velocity of drive wheel
    maxYaw = 2*Math.PI;   // max rad/s for chassis rotation rate

       
    // set PID coefficients
    
     m_BackLeftDrivePID.setP(kdP);
     m_BackLeftDrivePID.setI(kdI);
     m_BackLeftDrivePID.setD(kdD);
     m_BackLeftDrivePID.setIZone(kdIz);
     m_BackLeftDrivePID.setFF(kdFF);
     m_BackLeftDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

     m_BackRightDrivePID.setP(kdP);
     m_BackRightDrivePID.setI(kdI);
     m_BackRightDrivePID.setD(kdD);
     m_BackRightDrivePID.setIZone(kdIz);
     m_BackRightDrivePID.setFF(kdFF);
     m_BackRightDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

     m_FrontLeftDrivePID.setP(kdP);
     m_FrontLeftDrivePID.setI(kdI);
     m_FrontLeftDrivePID.setD(kdD);
     m_FrontLeftDrivePID.setIZone(kdIz);
     m_FrontLeftDrivePID.setFF(kdFF);
     m_FrontLeftDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

     m_FrontRightDrivePID.setP(kdP);
     m_FrontRightDrivePID.setI(kdI);
     m_FrontRightDrivePID.setD(kdD);
     m_FrontRightDrivePID.setIZone(kdIz);
     m_FrontRightDrivePID.setFF(kdFF);
     m_FrontRightDrivePID.setOutputRange(kdMinOutput, kdMaxOutput);

    /*m_FrontLeftTurnEncoder.setPosition(0);
    m_FrontRightTurnEncoder.setPosition(0);
    m_BackLeftTurnEncoder.setPosition(0);
    m_BackRightTurnEncoder.setPosition(0);*/
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    SmartDashboard.putNumber("Gyro", (m_gyro.getAngle()+180));    
    
    //Get the Direction input from the XBox controller left stick
    xAxis = MathUtil.applyDeadband(-m_controller.getLeftY(), .2)*maxVel; // linear m/s
    yAxis = MathUtil.applyDeadband(-m_controller.getLeftX(), .2)*maxVel; // linear m/s
    kYawRate = MathUtil.applyDeadband(-m_controller.getRightX(), .2)*maxYaw; // rad/s
    o_lyAxis = MathUtil.applyDeadband(-o_controller.getLeftY(), .2); // linear m/s
    o_ryAxis = MathUtil.applyDeadband(-o_controller.getRightY(), .2); // linear m/s

    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.2))
            * maxVel;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.2))
            * maxVel;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.2))
          *maxYaw;

      /*Calculate Swerve Module States based on controller readings
      if left bumper is pressed Drive will be Robot Relative
      otherwise Drive is Field Centric
      */

      if(m_controller.getLeftBumper()){
        fieldspeeds =  new ChassisSpeeds(xAxis, yAxis, rot);
      }
      else {
        fieldspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()+180));
      }
    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(fieldspeeds);
    SwerveModuleState BackLeftSwerve = moduleStates[0];
    SwerveModuleState BackRightSwerve = moduleStates[1];
    SwerveModuleState FrontLeftSwerve = moduleStates[2];
    SwerveModuleState FrontRightSwerve = moduleStates[3];
    var BackLeftOptimized = SwerveModuleState.optimize(BackLeftSwerve, Rotation2d.fromDegrees(m_BackLeftTurnEncoder.getPosition()));
    var BackRightOptimized = SwerveModuleState.optimize(BackRightSwerve, Rotation2d.fromDegrees(m_BackRightTurnEncoder.getPosition()));
    var FrontLeftOptimized = SwerveModuleState.optimize(FrontLeftSwerve, Rotation2d.fromDegrees(m_FrontLeftTurnEncoder.getPosition()));
    var FrontRightOptimized = SwerveModuleState.optimize(FrontRightSwerve, Rotation2d.fromDegrees(m_FrontRightTurnEncoder.getPosition()));
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method and the reference value of the optimized swerve mfodule states
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type, can be set to one of four 
     * parameters:
     * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    
    //Set actual motor output values to drive
    m_BackRightTurnPID.setReference(BackRightOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    m_FrontRightTurnPID.setReference(FrontRightOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    m_FrontLeftTurnPID.setReference(FrontLeftOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    m_BackLeftTurnPID.setReference(BackLeftOptimized.angle.getDegrees(), CANSparkMax.ControlType.kPosition);  
    m_BackLeftDrivePID.setReference(BackLeftOptimized.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_BackRightDrivePID.setReference(BackRightOptimized.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_FrontLeftDrivePID.setReference(FrontLeftOptimized.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_FrontRightDrivePID.setReference(FrontRightOptimized.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        
        
    /* //TURN DRIVE MOTORS OFF
    BackLeftDrive.set(0);
    BackRightDrive.set(0);
    FrontRightDrive.set(0);
    FrontLeftDrive.set(0);
    */

   //output the current values to the SmartDashboard, by reading BackLeftDrive and BackLeftTurn
    SmartDashboard.putNumber("P Gain", m_BackLeftDrivePID.getP());
    SmartDashboard.putNumber("Set Chassis Speed", Math.sqrt(xAxis*xAxis + yAxis*yAxis));
    SmartDashboard.putNumber("Set Chassis Angle", Math.atan2(yAxis,xAxis));
    SmartDashboard.putNumber("BL S_Angle", BackLeftSwerve.angle.getDegrees()); 
    SmartDashboard.putNumber("BL S_Velocity", BackLeftSwerve.speedMetersPerSecond);
    SmartDashboard.putNumber("BL P_Angle", m_BackLeftTurnEncoder.getPosition());
    SmartDashboard.putNumber("BL P_Velocity", m_BackLeftDriveEncoder.getVelocity());
    SmartDashboard.putNumber("BR S_Angle", BackRightSwerve.angle.getDegrees());
    SmartDashboard.putNumber("BR S_Velocity", BackRightSwerve.speedMetersPerSecond);
    SmartDashboard.putNumber("BR P_Angle", m_FrontRightTurnEncoder.getPosition());
    SmartDashboard.putNumber("BR P_Velocity", m_BackRightDriveEncoder.getVelocity());
    SmartDashboard.putNumber("FL S_Angle", FrontLeftSwerve.angle.getDegrees());
    SmartDashboard.putNumber("FL S_Velocity", FrontLeftSwerve.speedMetersPerSecond);
    SmartDashboard.putNumber("FL P_Angle", m_FrontRightTurnEncoder.getPosition());
    SmartDashboard.putNumber("FL P_Velocity", m_FrontLeftDriveEncoder.getVelocity());
    SmartDashboard.putNumber("FR S_Angle", FrontRightSwerve.angle.getDegrees());
    SmartDashboard.putNumber("FR S_Velocity", FrontRightSwerve.speedMetersPerSecond);
    SmartDashboard.putNumber("FR P_Angle", m_FrontRightTurnEncoder.getPosition());
    SmartDashboard.putNumber("FR P_Velocity", m_FrontRightDriveEncoder.getVelocity());


      
    //Run Intake
    intakeBot.set((m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis())*0.7);
    intakeTop.set(m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis());
    
    hangLeft.set(o_lyAxis*0.7);
    hangRight.set(o_ryAxis*0.7);

    //Set Shooter Speeds for Speaker or Amp Load
    if(o_controller.getXButton()){ //shooter for reverse
      //shooterLeft.set(.2);
      //shooterRight.set(-0.2);
      m_ShooterLeftPID.setReference(0.1 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
      m_ShooterRightPID.setReference(0.1 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
    }
    else if(o_controller.getAButton()){  //shooter speaker
      //shooterLeft.set(-0.7);
      //shooterRight.set(0.4);  
      m_ShooterLeftPID.setReference(-0.6 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
      m_ShooterRightPID.setReference(-0.3 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);  
    }
    else if(o_controller.getYButton()){  //slow shooter to load for amp/trap
      //shooterLeft.set(-0.2);`
      //shooterRight.set(0.2);
      m_ShooterLeftPID.setReference(-0.2 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
      m_ShooterRightPID.setReference(-0.2 * maxShootVelocity, CANSparkMax.ControlType.kVelocity);
    }
    else{
     //shooterLeft.set(0.0);  //shooter stops
     //shooterRight.set(0.0);
     m_ShooterLeftPID.setReference(0, CANSparkMax.ControlType.kVelocity);
     m_ShooterRightPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    if(o_controller.getRightBumper()){
      m_ShooterAnglePID.setReference(traplocation, CANSparkMax.ControlType.kPosition);
    }
    else if(o_controller.getBButton()){
      m_ShooterAnglePID.setReference(traplocation-7, CANSparkMax.ControlType.kPosition);
    }
    else {
      m_ShooterAnglePID.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    if(m_controller.getAButton()){
      m_gyro.reset();
    }

    if(m_controller.getRightBumper()){
      maxVel = 1.5;
    }
    else{
      maxVel = 4;
    }
//60 degrees

  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    System.out.println(m_BackLeftTurnEncoder.getPosition());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
