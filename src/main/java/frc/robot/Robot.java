/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private DifferentialDrive m_myRobot;
 
  public NeutralMode brake = NeutralMode.Brake;

  private static final int shooterCANID_1 = 5;
  private static final int shooterCANID_2 = 6;
  
  CANEncoder shootEncoder1;
  CANEncoder shootEncoder2;

  private Solenoid a_collector;

  private double gR = 10.25641025;
  private double circumference = 6 * Math.PI;
  private boolean collecting = true;
  private boolean s_ultra1Range = false;
  private boolean s_ultra2Range = false;
  private boolean togglecollector = false;
  private CANSparkMax m_shooterright;
  private CANSparkMax m_shooterleft;
  
  private CANPIDController p_shooter;

  Spark m_hood;
  Spark m_feeder; 
  private double gRCombin = circumference/gR;
  

  private Ultrasonic s_ultra1;
  private Ultrasonic s_ultra2;

  private WPI_TalonFX m_talon1;
  private WPI_TalonFX m_talon2;
  private WPI_TalonFX m_talon3;
  private WPI_TalonFX m_talon4;
  private WPI_TalonFX m_talon5;
  private WPI_TalonFX m_talon6;

  final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise;
  XboxController driveController = new XboxController(0);
  XboxController operateController = new XboxController(1);
  Compressor comp = new Compressor(0);
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {


//Drive set

  m_talon1 = new WPI_TalonFX(1);
  m_talon2 = new WPI_TalonFX(2); 
  m_talon3 = new WPI_TalonFX(3);
  m_talon4 = new WPI_TalonFX(4);
  m_talon5 = new WPI_TalonFX(5);
  m_talon6 = new WPI_TalonFX(6);
  
  m_shooterleft = new CANSparkMax(shooterCANID_1, MotorType.kBrushless);
  m_shooterright = new CANSparkMax(shooterCANID_2, MotorType.kBrushless);
  m_shooterright.follow(m_shooterleft);
  m_shooterright.setInverted(true);


  s_ultra1 = new Ultrasonic(0, 1);
  s_ultra2 = new Ultrasonic(2, 3);
  
  a_collector = new Solenoid(0);
   


  final SpeedControllerGroup left = new SpeedControllerGroup(m_talon1, m_talon2, m_talon5);
  final SpeedControllerGroup right = new SpeedControllerGroup(m_talon3, m_talon4, m_talon6);

  shootEncoder1 = new CANEncoder(m_shooterleft);
  shootEncoder2 = new CANEncoder(m_shooterright); 

  p_shooter = new CANPIDController(m_shooterleft);
  p_shooter.setFeedbackDevice(shootEncoder1);

  m_myRobot = new DifferentialDrive(left, right);
  m_talon1.setInverted(false);
  m_talon2.setInverted(false);
  m_talon5.setInverted(false);
  m_talon3.setInverted(true);
  m_talon4.setInverted(true);
  m_talon6.setInverted(true);  

  m_talon1.setNeutralMode(brake);
  m_talon2.setNeutralMode(brake);
  m_talon3.setNeutralMode(brake);
  m_talon4.setNeutralMode(brake);
  m_talon5.setNeutralMode(brake);  
  m_talon6.setNeutralMode(brake);

  m_myRobot.setMaxOutput(0.9);
  m_talon1.setInverted(kInvertType);
  m_talon2.setInverted(kInvertType);
  m_talon5.setInverted(kInvertType);
 
 }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_talon1.setSelectedSensorPosition(0);
    m_talon2.setSelectedSensorPosition(0);
    m_talon3.setSelectedSensorPosition(0);
    m_talon4.setSelectedSensorPosition(0);
    m_talon5.setSelectedSensorPosition(0);
    m_talon6.setSelectedSensorPosition(0);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double s_mleft = Math.abs(m_talon1.getSelectedSensorPosition() / 2048);
    double s_mright = Math.abs(m_talon4.getSelectedSensorPosition()/2048);
    double lwheelSpin = gRCombin * s_mleft; 
    double rwheelSpin = gRCombin * s_mright; //how many inches per motor spin 
    
   
    //Gear Ratio: 10.25641025
    
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        //blank inches forward once it is 12 inches before the point that we want to reach we will run the colletor and indexer mechanism in preperation to grab the two balls. 
        //At this point we will run them until the ultrasonic at the end of indexer is completely full.Then we will stop the indexer and the collector once this value is hit. Once this happens, we will reverse the function and move backwards a set amount of inches so we are at the point that we are at the ideal shooting range.
        //Then we will start a timer. We will use the limelight to align at this point and then we will spin the shooter up to the ideal speed at that distance. 
        //Once this is complete we will fire all 5 balls (OPTIONAL) at this speed hopefully dead center. After this is complete we will turn to the left so the collector is facing forwards. We will then drive forwards and then spin to the right and then go collect the ones on the end of the rendevous and then  
        if(lwheelSpin < 12 && rwheelSpin < 12) {
        m_myRobot.arcadeDrive(0.2, 0);
        } else if (lwheelSpin == 12 && rwheelSpin == 12) {
          m_myRobot.arcadeDrive(0, 0);
        }
        
        break;
    }
  }
  @Override
  public void teleopInit() {
    s_ultra1.setAutomaticMode(true);
  }

  @Override
  public void teleopPeriodic() {
  m_myRobot.arcadeDrive(driveController.getX(Hand.kRight), driveController.getY(Hand.kLeft));
 
  shootEncoder1 = new CANEncoder(m_shooterleft);
  shootEncoder2 = new CANEncoder(m_shooterright);


  //to be changed below based on actual robot values
  final double h2 = 90.875; //height of target
  final double h1 = 15.875; //height of limeligt

  final double a2 = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
  final double a1 = Math.toRadians(23); //angle of limelight

  final double heightValue = (h2 - h1);
  
  final double angleboth = a1 + a2;
  final double tanValue = Math.tan(angleboth);
  
shootEncoder1 = new CANEncoder(m_shooterleft);
  shootEncoder2 = new CANEncoder(m_shooterright);
    if(operateController.getYButton()){
      togglecollector = !togglecollector;
    }



  
  if(collecting){
    if(s_ultra1.getRangeInches() < 8){
        //run the index for however long
        s_ultra1Range = true;
    }else{
        s_ultra1Range = false;
    }
  }

  if(s_ultra2.getRangeInches() < 8){
    s_ultra2Range = true;
}else{
    s_ultra2Range = false;
}
 if(operateController.getRawAxis(2) > .5){
   //collector runs
   collecting = true;
 }else{
   collecting = false;
 }

 double shooter_speed = 1500.0;

 if(operateController.getRawAxis(3) > .5){
  p_shooter.setReference(shooter_speed, ControlType.kVelocity);
   //pid shit
 }else{
  p_shooter.setReference(0, ControlType.kVelocity);
 }
 //smart Dashboard
  SmartDashboard.putNumber("distance", heightValue/tanValue); //distance from target*/
  SmartDashboard.putNumber("Rotationleft1", m_talon1.getSelectedSensorPosition()/2048);
  SmartDashboard.putNumber("Rotationsleft2", m_talon2.getSelectedSensorPosition()/2048);
  SmartDashboard.putNumber("Rotationsright1", m_talon3.getSelectedSensorPosition()/2048);
  SmartDashboard.putNumber("Rotationsright2", m_talon4.getSelectedSensorPosition()/2048);
  SmartDashboard.putNumber("Rotationsleft3", m_talon5.getSelectedSensorPosition()/2048);
  SmartDashboard.putNumber("Rotationsright3", m_talon6.getSelectedSensorPosition()/2048);
  SmartDashboard.putNumber("NeoEncoder1", shootEncoder1.getVelocity());
  SmartDashboard.putNumber("NeoEncoder2", shootEncoder2.getVelocity()); 
  SmartDashboard.putNumber("NeoEncoder2ConversionFactor", shootEncoder2.getPositionConversionFactor()); 
  SmartDashboard.putNumber("NeoEncoder1ConversionFactor", shootEncoder1.getPositionConversionFactor());
  SmartDashboard.putNumber("ultra1", s_ultra1.getRangeInches());
  SmartDashboard.putBoolean("BallIndex", s_ultra1Range);
  SmartDashboard.putBoolean("Index FULL", s_ultra2Range);

  
  
  //double distance = 3; //ft
//  p_shooter.setReference(distance, ControlType.kPosition);

  }
  // C/GR = , RPM Encoder, Distacne per rotation of Motor
  @Override
  public void testInit() {
   comp.start();
   }
   
  @Override
  public void testPeriodic() {
   if(operateController.getRawButton(2)) {
     comp.start();
   }
    if(operateController.getRawButton(1)) {
      comp.stop();
    }
  }


    
}
