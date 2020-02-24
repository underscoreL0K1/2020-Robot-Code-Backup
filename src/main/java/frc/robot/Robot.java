/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
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

  private static final int shooterCANID_1 = 11;
  private static final int shooterCANID_2 = 14;
  private static final int collectCANID = 9;
  private static final int indexerCANID = 10;
  private static final int feederCANID = 8;
  private static final int winchCANID_1 = 0;  //Neos
  private static final int winchCANID_2 = 0;
  private static final int hoodCANID = 7;


  
  private CANEncoder shootEncoder1;
  private CANEncoder shootEncoder2;



  private Solenoid a_collector;

  private int t_indexreset;

  private double gR = 10.25641025;
  private double circumference = 6 * Math.PI;
  private boolean collecting = true;
  private boolean s_ultra1Range = false;
  private boolean s_ultra2Range = false;
  private boolean togglecollector = false;
  private CANSparkMax m_shooterright;
  private CANSparkMax m_shooterleft;
  private WPI_VictorSPX m_feeder; 
  private WPI_VictorSPX m_hood;
  private WPI_VictorSPX m_collector;
  private WPI_TalonSRX m_indexer;
  private Gyro s_roboGyro;
  
  private CANPIDController p_shooter;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;



  
  private double gRCombin = circumference/gR;
  
  private Timer t_timer;
  private Timer t_timer2;

  private Ultrasonic s_ultra1;
  private Ultrasonic s_ultra2;

  private int ballcount = 0;

  private WPI_TalonFX m_talon1;
  private WPI_TalonFX m_talon2;
  private WPI_TalonFX m_talon3;
  private WPI_TalonFX m_talon4;
  private WPI_TalonFX m_talon5;
  private WPI_TalonFX m_talon6;
  private AnalogPotentiometer s_hood;

  final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise;
  XboxController driveController = new XboxController(0);
  XboxController operateController = new XboxController(1);
  Compressor comp = new Compressor(0);
  final int kTimeoutMs = 30;
  
Boolean limeHasTarget = false;
double limeTarget;
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

  s_hood = new AnalogPotentiometer(0);

 // m_indexer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
  m_collector = new WPI_VictorSPX(collectCANID);
  m_indexer = new WPI_TalonSRX(indexerCANID);
  
  m_shooterleft = new CANSparkMax(shooterCANID_1, MotorType.kBrushless);
  m_shooterright = new CANSparkMax(shooterCANID_2, MotorType.kBrushless);
  m_shooterleft.setInverted(true);
  m_shooterright.follow(m_shooterleft, true);
 

  m_feeder = new WPI_VictorSPX(feederCANID);
  m_hood = new WPI_VictorSPX(hoodCANID); 
  s_ultra1 = new Ultrasonic(8, 9);
  s_ultra2 = new Ultrasonic(6, 7);
  
  s_roboGyro = new Gyro(){
  
    @Override
    public void close() throws Exception {
      // TODO Auto-generated method stub
      
    }
  
    @Override
    public void reset() {
      // TODO Auto-generated method stub
      
    }
  
    @Override
    public double getRate() {
      // TODO Auto-generated method stub
      return 0;
    }
  
    @Override
    public double getAngle() {
      // TODO Auto-generated method stub
      return 0;
    }
  
    @Override
    public void calibrate() {
      // TODO Auto-generated method stub
      
    }
  };
  
  a_collector = new Solenoid(0);

  p_shooter = m_shooterleft.getPIDController();
  shootEncoder1 = m_shooterleft.getEncoder();



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
  t_timer = new Timer();
  t_timer2 = new Timer(); 
   //PID!!!
   kP = 0; 
   kI = 0;
   kD = 0; 
   kIz = 0; 
   kFF = 0.000156; 
   kMaxOutput = 1; 
   kMinOutput = 0;
   maxRPM = 5700;
 
   // Smart Motion Coefficients
   maxVel = 4000; // rpm
   maxAcc = 1500;
 
   p_shooter.setP(kP);
   p_shooter.setI(kI);
   p_shooter.setD(kD);
     p_shooter.setIZone(kIz);
     p_shooter.setFF(kFF);
     p_shooter.setOutputRange(kMinOutput, kMaxOutput);
  
   int smartMotionSlot = 0;
   p_shooter.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
   p_shooter.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
   p_shooter.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
   p_shooter.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
 
   // display PID coefficients on SmartDashboard
   SmartDashboard.putNumber("P Gain", kP);
   SmartDashboard.putNumber("I Gain", kI);
   SmartDashboard.putNumber("D Gain", kD);
   SmartDashboard.putNumber("I Zone", kIz);
   SmartDashboard.putNumber("Feed Forward", kFF);
   SmartDashboard.putNumber("Max Output", kMaxOutput);
   SmartDashboard.putNumber("Min Output", kMinOutput);
 
   // display Smart Motion coefficients
   SmartDashboard.putNumber("Max Velocity", maxVel);
   SmartDashboard.putNumber("Min Velocity", minVel);
   SmartDashboard.putNumber("Max Acceleration", maxAcc);
   SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
   SmartDashboard.putNumber("Set Position", 0);
   SmartDashboard.putNumber("Set Velocity", 4000);
 
    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
 
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

   public void limelightTracking(){
    double steer = 0.06; 
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
if (tv < 1) {
limeHasTarget = false;
} else {
  limeHasTarget = true;
}


    double limeTarget = tx * steer; 



   }

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
    m_feeder.setSelectedSensorPosition(0);
    s_roboGyro.reset();
    t_timer.reset(); 
    t_timer.start();

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
    int state = 0;
    
   
    
   
    //Gear Ratio: 10.25641025
    
    switch (m_autoSelected) {
      case kCustomAuto:
       
        break;
      case kDefaultAuto:
      default:
      if(state == 0){
        //run shooter
        //align with target
        if(s_roboGyro.getAngle() < -20){
          //stop drive
          state = 1;
        }
      }
     if((state == 1)) {

     //run feeder and index == shoots + limeligaht
        
        if(t_timer.get() > 4){
          t_timer.stop();
          state = 2;
        }
     } else if (state == 2){
       //set motors to turn right
       if(s_roboGyro.getAngle()  > 0){
        //stop drive motrs turn
        //drive back and collect
        //index when ultrasonic
        // as soon as encoders hit their mark, 
                                              state = 3;
       }else if (state == 3) {
        //move backwards with encoder values to shooting position
        //go to state 4 
       } else if (state == 4) { 
         //shoot timers and indexer stuff + limelight  t_timer_2 
         //TELEOP!!!
      }
     }
      
      
      //[      ] inches forward once it is 12 inches before the point that we want to reach we will run the colletor and indexer mechanism in preperation to grab the two balls. 
        //At this point we will run them until the ultrasonic at the end of indexer is completely full.Then we will stop the indexer and the collector once this value is hit. Once this happens, we will reverse the function and move backwards a set amount of inches so we are at the point that we are at the ideal shooting range.
        //Then we will start a timer. We will use the limelight to align at this point and then we will spin the shooter up to the ideal speed at that distance. 
        //Once this is complete we will fire all 5 balls. (OPTIONAL) at this speed hopefully dead center. After this is complete we will turn to the left so the collector is facing forwards. We will then drive forwards and then spin to the right and then go collect the ones on the end of the rendevous and then. 

        //Auto 2
        //Already Lined at start. Rev shoot and feed and fire. straighten, backwards while collecting and indexing into. Then stop when the last one hits the back but this would be tough but we have to regulte when the actual things are moving. When we get to it. 
  /*if ((l_wheelspin.get() > 0.5) && (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1)){
    double kP_turn; 
    double min_command;
    double error = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double steering_change = 0;    
    kP_turn = -.013; 
    min_command = .08;
        if(lwheelSpin < 12 && rwheelSpin < 12) {
        m_myRobot.arcadeDrive(0.2, 0);
        } else if (lwheelSpin == 12 && rwheelSpin == 12) {
          m_myRobot.arcadeDrive(0, 0);
        }
      }*/
        break;
    }
  }
  @Override
  public void teleopInit() {
    s_ultra1.setAutomaticMode(true);
    //m_indexer.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    
  }

  @Override
  public void teleopPeriodic() {
 
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


    if(togglecollector){
      a_collector.set(true);
      
    }else{
      a_collector.set(false);
    }

    if(operateController.getRawAxis(3) > 0.5){
    m_feeder.set(-0.7);
    m_indexer.set(0.7); 
    ballcount = 0;
    }else  if(operateController.getBumper(Hand.kLeft)){
m_indexer.set(-0.3);
m_collector.set(-1);
m_feeder.set(.7);
 }else if(operateController.getBumper(Hand.kRight)){
m_indexer.set(0.3);
m_collector.set(1);
 }else{
m_indexer.set(0);
m_feeder.set(0);

 }
    

    m_hood.set(operateController.getRawAxis(1)*0.25);



  
  //if(collecting){
    if(s_ultra1.getRangeInches() < 10){
        //run the index for however long
        s_ultra1Range = true;
    }else{
       s_ultra1Range = false;
       t_indexreset = 0;
    }
 // }


  if(s_ultra1Range){
    if(t_indexreset == 1){
      ballcount++;
      t_indexreset++;
    }else{
      t_indexreset++;
    }
  }

  if(s_ultra2.getRangeInches() < 8){
    s_ultra2Range = true;
  }else{
    s_ultra2Range = false;
  } 
 /*if(operateController.getAButton()){
  m_collector.set(0.5);
   collecting = true;
 }else if(operateController.getBButton()){
  m_collector.set(-0.5);
 }else{
   m_collector.set(0);
   collecting = false;
   
 }
*/
limelightTracking();
double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
double hordis = Math.abs(tx);
double steer = 0.06; 
double limeTarget = tx * steer; 
if(driveController.getRawAxis(3) > 0.7) {
    if(limeHasTarget = true){ //has a target
      if (hordis > 1){
        m_myRobot.arcadeDrive(limeTarget, 0);
      }else if(hordis < 1) {
        m_myRobot.arcadeDrive(0, 0);
      }
    }
  }else{
    m_myRobot.arcadeDrive((driveController.getX(Hand.kRight)), -(driveController.getY(Hand.kLeft)));

  }

    
if(operateController.getAButton()){
  m_collector.set(1);
} else if(operateController.getBButton()) {
  m_collector.set(-1);
}else{
  m_collector.set(0);
}

 

 



 //double shooter_speed = 1500.0;
 //smart Dashboard
  SmartDashboard.putNumber("distance", heightValue/tanValue); //distance from target*/
  SmartDashboard.putNumber("Pot", ((s_hood.get() * 163.429271856365)-2.603118));
  SmartDashboard.putNumber("index encoder", m_indexer.getSensorCollection().getQuadraturePosition());
  SmartDashboard.putNumber("NeoEncoder1", shootEncoder1.getVelocity());
  SmartDashboard.putNumber("NeoEncoder2", shootEncoder2.getVelocity()); 
  SmartDashboard.putNumber("ultra1", s_ultra1.getRangeInches());
  SmartDashboard.putBoolean("BallIndex", s_ultra1Range);
  SmartDashboard.putBoolean("Index FULL", s_ultra2Range);
  SmartDashboard.putNumber("Gyro Angle", s_roboGyro.getAngle()); 
  SmartDashboard.putNumber("Ballcount", ballcount);
  SmartDashboard.putNumber("Index reset Timer", t_indexreset);
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    
    
   // if PID coefficients on SmartDashboard have changed, write new values to controller
   if((p != kP)) { p_shooter.setP(p); kP = p; }
   if((i != kI)) { p_shooter.setI(i); kI = i; }
   if((d != kD)) { p_shooter.setD(d); kD = d; }
   if((iz != kIz)) { p_shooter.setIZone(iz); kIz = iz; }
   if((ff != kFF)) { p_shooter.setFF(ff); kFF = ff; }
   if((max != kMaxOutput) || (min != kMinOutput)) { 
     p_shooter.setOutputRange(min, max); 
     kMinOutput = min; kMaxOutput = max; 
   }
   if((maxV != maxVel)) { p_shooter.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
   if((minV != minVel)) { p_shooter.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
   if((maxA != maxAcc)) { p_shooter.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
   if((allE != allowedErr)) { p_shooter.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
  
    double setPoint, processVariable;
    setPoint = 0;
      
      if(operateController.getRawAxis(2) > 0.5) {
        setPoint = SmartDashboard.getNumber("Set Velocity", 4000);
      }
        /*else {
        setPoint = SmartDashboard.getNumber("Set Position", 0);
        /**
         * As with other PID modes, Smart Motion is set by calling the
         * setReference method on an existing pid object and setting
         * the control type to kSmartMotion
          p_shooter.setReference(setPoint, ControlType.kSmartMotion);
        processVariable = shootEncoder1.getPosition();
      }*/
      p_shooter.setReference(setPoint, ControlType.kVelocity);
      processVariable = shootEncoder1.getVelocity();
      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("Process Variable", processVariable);
      SmartDashboard.putNumber("Output", m_shooterleft.getAppliedOutput());
    }
  
  //double distance = 3; //ft
//p_shooter.setReference(distance, ControlType.kPosition);

  
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
