/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
 
  public NeutralMode brake;

  private static final int shooterCANID_1 = 11;
  private static final int shooterCANID_2 = 14;
  private static final int collectCANID = 9;
  private static final int indexerCANID = 10;
  private static final int feederCANID = 8;
  private static final int winchCANID_1 = 13;  //Neos
  private static final int winchCANID_2 = 12;
  private static final int hoodCANID = 7;

  private CANEncoder shootEncoder1;
  private CANEncoder shootEncoder2;

  private DoubleSolenoid a_collector; 
  private DoubleSolenoid a_bigboypiston;
  private Solenoid a_pancake;

  private int t_indexreset;

  private double gR = 10.25641025;
  private double circumference = 6 * Math.PI;
  private boolean collecting = true;
  private boolean s_ultra1Range = false;
  private boolean s_ultra2Range = false;
  
  
  private SpeedControllerGroup left; 
  private SpeedControllerGroup right; 

  private boolean togglecollector = false;
  private CANSparkMax m_shooterright;
  private CANSparkMax m_shooterleft;
  private CANSparkMax m_climbleft;
  private CANSparkMax m_climbright;
  private WPI_VictorSPX m_feeder; 
  private WPI_VictorSPX m_hood;
  private WPI_VictorSPX m_collector;
  private WPI_TalonSRX m_indexer;
  private Gyro s_roboGyro;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


  
  private double gRCombin = circumference/gR;
  
  private int t_ultra1;

  private Timer t_timer;
  private Timer t_timer2;
  private Timer t_auto; 
 
  private boolean inRANGE;

  private Ultrasonic s_ultra1;
  private Ultrasonic s_ultra2;
  private CANPIDController m_pidController;
  private int ballcount = 0;

  private WPI_TalonFX m_talon1;
  private WPI_TalonFX m_talon2;
  private WPI_TalonFX m_talon3;
  private WPI_TalonFX m_talon4;
  private WPI_TalonFX m_talon5;
  private WPI_TalonFX m_talon6;
  private AnalogPotentiometer s_hood;

  private int smartMotionSlot;

  

  final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise;
  XboxController driveController = new XboxController(0);
  XboxController operateController = new XboxController(1);
  Compressor comp = new Compressor(0);
  private boolean launch;
  
//Boolean limeHasTarget = false;
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

  m_collector = new WPI_VictorSPX(collectCANID);
  m_indexer = new WPI_TalonSRX(indexerCANID);
  m_climbleft = new CANSparkMax(winchCANID_1, MotorType.kBrushless);
  m_climbright = new CANSparkMax(winchCANID_2, MotorType.kBrushless);
  
  
  m_shooterleft = new CANSparkMax(shooterCANID_1, MotorType.kBrushless);
  m_shooterright = new CANSparkMax(shooterCANID_2, MotorType.kBrushless);
  
  brake = NeutralMode.Brake;

  m_shooterleft.setInverted(true);
  m_shooterright.follow(m_shooterleft, true);
 

  m_feeder = new WPI_VictorSPX(feederCANID);
  m_hood = new WPI_VictorSPX(hoodCANID); 
  s_ultra1 = new Ultrasonic(7, 6);
  s_ultra2 = new Ultrasonic(8, 9);
  
  
  
  a_collector = new DoubleSolenoid(2, 3); 
  a_bigboypiston = new DoubleSolenoid(0, 7);
  a_pancake = new Solenoid(1);


  shootEncoder1 = m_shooterleft.getEncoder();
  shootEncoder2 = m_shooterright.getEncoder();

  // PID coefficients
 
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

  left = new SpeedControllerGroup(m_talon1, m_talon2, m_talon5);
  right = new SpeedControllerGroup(m_talon3, m_talon4, m_talon6);

  

  
  

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
  t_ultra1 = 0;
   
 
   
 
 }
 public void limelightTracking(){
  
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
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

   /*public void limelightTracking(){
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
*/
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
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

    m_pidController = m_shooterleft.getPIDController();
    kP = .002; 
  kI = 0.0;
  kD = 0.002; 
  kIz = 0; 
  kFF = (10/5700) * 4000; 
  kMaxOutput = 1; 
  kMinOutput = 0;
  maxRPM = 4000;

  // set PID coefficients
  m_pidController.setP(kP);
  m_pidController.setI(kI);
  m_pidController.setD(kD);
  m_pidController.setIZone(kIz);
  m_pidController.setFF(kFF);
  m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  // display PID coefficients on SmartDashboard
  SmartDashboard.putNumber("P Gain", kP);
  SmartDashboard.putNumber("I Gain", kI);
  SmartDashboard.putNumber("D Gain", kD);
  SmartDashboard.putNumber("I Zone", kIz);
  SmartDashboard.putNumber("Feed Forward", kFF);
  SmartDashboard.putNumber("Max Output", kMaxOutput);
  SmartDashboard.putNumber("Min Output", kMinOutput);
  t_auto.start();

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

      // read PID coefficients from SmartDashboard
  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { m_pidController.setP(p); kP = p; }
  if((i != kI)) { m_pidController.setI(i); kI = i; }
  if((d != kD)) { m_pidController.setD(d); kD = d; }
  if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
  if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { 
    m_pidController.setOutputRange(min, max); 
    kMinOutput = min; kMaxOutput = max; 
  }
    double setPoint = operateController.getRawAxis(1)*maxRPM;
  
    //System.out.println(shootEncoder1.getVelocity());
    double tx = (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    double hordis = Math.abs(tx);
    double steer = .055; 
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", shootEncoder1.getVelocity());
    SmartDashboard.putNumber("ProcessVariable", shootEncoder2.getVelocity());

    
        
        if(t_auto.get() > 0 && t_auto.get() < 8.5){
        m_pidController.setReference(4000 , ControlType.kVelocity); 
        } else {
        m_pidController.setReference(0, ControlType.kVelocity); 
        }
        if(t_auto.get() > 0 && t_auto.get() < 8.5 && (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1)) {
          if (hordis > 1 || hordis < -1) {
          if(tx < 5 && tx > -5){
                m_myRobot.arcadeDrive((tx * .1 ), 0);
              }else{
                m_myRobot.arcadeDrive(limeTarget, 0);
              }    
            }
          }
       
        if(t_auto.get() > 4.5 && t_auto.get() < 8.5){
          m_feeder.set(-1);
          m_indexer.set(1); 
        } 
        if(t_auto.get() > 8.5 && t_auto.get() < 11) {
          m_myRobot.arcadeDrive(-0.6, 0);
        }
        t_auto.stop();
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
    s_ultra2.setAutomaticMode(true);
    t_ultra1 = 0;
    //m_indexer.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    comp.stop();
    m_pidController = m_shooterleft.getPIDController();
  kP = .002; 
  kI = 0.0;
  kD = 0.002; 
  kIz = 0; 
  kFF = (10/5700) * 4000; 
  kMaxOutput = 1; 
  kMinOutput = 0;
  maxRPM = 4000;

  inRANGE = false;


  // set PID coefficients
  m_pidController.setP(kP);
  m_pidController.setI(kI);
  m_pidController.setD(kD);
  m_pidController.setIZone(kIz);
  m_pidController.setFF(kFF);
  m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  // display PID coefficients on SmartDashboard
  SmartDashboard.putNumber("P Gain", kP);
  SmartDashboard.putNumber("I Gain", kI);
  SmartDashboard.putNumber("D Gain", kD);
  SmartDashboard.putNumber("I Zone", kIz);
  SmartDashboard.putNumber("Feed Forward", kFF);
  SmartDashboard.putNumber("Max Output", kMaxOutput);
  SmartDashboard.putNumber("Min Output", kMinOutput);

  }

  @Override
  public void teleopPeriodic() {
  
  
  t_ultra1++;
  
  


  //to be changed below based on actual robot values
  final double h2 = 90.875; //height of target
  final double h1 = 14.75; //height of limeligt

  final double a2 = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
  final double a1 = Math.toRadians(13); //angle of limelight

  final double heightValue = (h2 - h1);
  
  final double angleboth = a1 + a2;
  final double tanValue = Math.tan(angleboth);

  double distanceFromTarget = heightValue/tanValue;


  double hoodAngle;
  double calculateAngle;


  // read PID coefficients from SmartDashboard
  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { m_pidController.setP(p); kP = p; }
  if((i != kI)) { m_pidController.setI(i); kI = i; }
  if((d != kD)) { m_pidController.setD(d); kD = d; }
  if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
  if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { 
    m_pidController.setOutputRange(min, max); 
    kMinOutput = min; kMaxOutput = max; 
  }
    double setPoint = operateController.getRawAxis(1)*maxRPM;
  if (operateController.getRawAxis(2) > 0.7){
    m_pidController.setReference(5200 , ControlType.kVelocity);
  } else {
    m_pidController.setReference(0 , ControlType.kVelocity);
  }
   // System.out.println(shootEncoder1.getVelocity());
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", shootEncoder1.getVelocity());
    SmartDashboard.putNumber("ProcessVariable", shootEncoder2.getVelocity());
  
 if(operateController.getYButtonPressed() || driveController.getRawButton(6)) {
      a_collector.set(Value.kForward);
    } else if(operateController.getAButtonPressed() || driveController.getRawButton(5)){
      a_collector.set(Value.kReverse);
    } else {
   
      a_collector.set(Value.kOff);
    }

  if(driveController.getAButtonPressed()) {
   a_bigboypiston.set(Value.kForward); 
 } else if (driveController.getBButtonPressed()) {
   a_bigboypiston.set(Value.kReverse);
 } else {
   a_bigboypiston.set(Value.kOff); 
 }
 SmartDashboard.putBoolean("Ready to Launch", launch); 
  if(driveController.getXButtonPressed() && driveController.getAButton() || operateController.getXButton()) {
  a_pancake.set(true);
  launch = false;
}else if(driveController.getYButtonPressed()||operateController.getXButtonReleased()){
  launch = true; 
  a_pancake.set(false);
}

  if(operateController.getRawAxis(3) > 0.5){
    m_feeder.set(-1);
    m_indexer.set(.7); 
    ballcount = 0;
  }else if(operateController.getBumper(Hand.kLeft)){
    m_indexer.set(-0.3);
    m_collector.set(-1);
    m_feeder.set(.7);
 }else if(operateController.getBumper(Hand.kRight)){
    m_indexer.set(0.3);
    m_collector.set(1);
 }else if(s_ultra1.getRangeInches() < 5 && !(s_ultra2.getRangeInches() < 5 )){
  m_indexer.set(.4);
  collecting = true;
}else { 
  if(t_ultra1 < 5){
    m_feeder.set(-.2);
  } else if(collecting && s_ultra2.getRangeInches() < 8){
    t_ultra1 = 0;
  } else {
    collecting = false;
    m_feeder.set(0);                                     
  }
  m_indexer.set(0);
 }
  if (operateController.getPOV() == 0) {
    m_climbleft.set(0.5);
    m_climbright.set(-0.5); 
  } else if(operateController.getPOV() == 180){
    m_climbleft.set(-0.5); 
    m_climbright.set(0.5); 
  } else {
    m_climbleft.set(0);
    m_climbright.set(0);
  }

 if (driveController.getRawAxis(3) > 0.65){
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
} else if (driveController.getRawAxis(3) < 0.6) {
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
}





 if(s_ultra1Range){
    if(t_indexreset == 1){
      ballcount++;
      t_indexreset++;
    }else{
      t_indexreset++;
    }
  }

 if(operateController.getAButton()){
  m_collector.set(0.7);
   collecting = true;
 }else if(operateController.getBButton()){
  m_collector.set(-0.5);
 }else{
   m_collector.set(0);
   collecting = false;
   
 }


limelightTracking();
double tx = (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
double hordis = Math.abs(tx);
double steer = .055; 
limeTarget = tx * steer; 
if(driveController.getRawAxis(3) > 0.7 && (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1)) {
  if (hordis > 1 || hordis < -1) {
  if(tx < 5 && tx > -5){
        m_myRobot.arcadeDrive((tx * .1 ), 0);
      }else{
        m_myRobot.arcadeDrive(limeTarget, 0);
      }    
    }
  }else{
   
      m_myRobot.arcadeDrive((driveController.getX(Hand.kRight)), -(driveController.getY(Hand.kLeft)));

  }
calculateAngle = 1/*equation*/;
hoodAngle = (((s_hood.get() * 163.429271856365)-2.603118) + 23); //place
 // if (hoodAngle > -0.25 && hoodAngle < 0.25) {
  //m_hood.set(operateController.getRawAxis(1)*0.25); 
  //} else if(driveController.getRawAxis(3) > 0.7 && hoodAngle > (calculateAngle+0.25)) {//changeable constant with the add
   // m_hood.set(-0.2);
  //}else if(driveController.getRawAxis(3) > 0.7 && hoodAngle < (calculateAngle -0.25)) {
   // m_hood.set(0.2);
  //} else{
 // m_hood.set(operateController.getRawAxis(1)*0.25);
//}
  
if(operateController.getRawButton(7)) {
  comp.start();
}
 if(operateController.getRawButton(8)) {
   comp.stop();
 }


if(distanceFromTarget > 160 && distanceFromTarget < 200){

  inRANGE = true;
} else {
  inRANGE = false;
}


 

 



 //double shooter_speed = 1500.0;
 //smart Dashboard
  SmartDashboard.putNumber("distance", heightValue/tanValue); //distance from target*/
  SmartDashboard.putNumber("Pot", (((s_hood.get() * 163.429271856365)-2.603118)) + 23); //angle of hood
  SmartDashboard.putNumber("index encoder", m_indexer.getSensorCollection().getQuadraturePosition());
  SmartDashboard.putNumber("NeoEncoder1", shootEncoder1.getVelocity());
  SmartDashboard.putNumber("NeoEncoder2", shootEncoder2.getVelocity()); 
  SmartDashboard.putNumber("ultra1", s_ultra1.getRangeInches());
  SmartDashboard.putBoolean("BallIndex", s_ultra1Range);
  SmartDashboard.putBoolean("Index FULL", s_ultra2Range);
  SmartDashboard.putNumber("Gyro Angle", s_roboGyro.getAngle()); 
  SmartDashboard.putNumber("Ballcount", ballcount);
  SmartDashboard.putNumber("Index reset Timer", t_indexreset);
  SmartDashboard.putBoolean("CANSHOOT", inRANGE);
}
  
  //double distance = 3; //ft
//p_shooter.setReference(distance, ControlType.kPosition);

  
  // C/GR = , RPM Encoder, Distacne per rotation of Motor
  @Override
  public void testInit() {
    //comp.start();
   }
   
  @Override
  public void testPeriodic() {
   
  }


    
}
