/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//pog
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
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

 int x;
  

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
  
  x = 0;

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

  m_myRobot.setMaxOutput(0.60);
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
   * This funcion is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  
    
  }

    @Override
  public void autonomousInit() {
    s_ultra1.setAutomaticMode(true);
    s_ultra2.setAutomaticMode(true);
    x = 0;
    m_talon1.setSelectedSensorPosition(0);
    m_talon4.setSelectedSensorPosition(0);
NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
 m_talon1.setSelectedSensorPosition(0);
 m_talon2.setSelectedSensorPosition(0);

 a_collector.set(Value.kReverse);
 AutoRed = true;
//super.autonomousInit();

  }
  
  @Override
  public void autonomousPeriodic() {
    double s_mleft = Math.abs(m_talon1.getSelectedSensorPosition() / 2048);
    double s_mright = Math.abs(m_talon4.getSelectedSensorPosition()/2048);
    double lwheelSpin = gRCombin * s_mleft; 
    double rwheelSpin = gRCombin * s_mright; //how many inches per motor spin 
    int state = 0;
    int phase = 1;

    System.out.println(rwheelSpin);

    int move5ft = 45;
   


    
    
   

    if(s_ultra1.getRangeInches() < 5 && !(s_ultra2.getRangeInches() < 5 )){
      m_indexer.set(.4);
    }
    else{
      m_indexer.set(0);
    }

    ultraAuto = s_ultra1.getRangeInches();
    SmartDashboard.putNumber("ultra1", s_ultra1.getRangeInches());
    SmartDashboard.putBoolean("Red Auto", AutoRed);

    if (rwheelSpin < 50 ) {
      left.set(-0.5);
      right.set(-0.5);
      m_collector.set(1);
      
      if(ultraAuto > 9){
        AutoRed = true;
      } //else {AutoRed = false;}


    }


    
      /*if(ultraAuto > 9){
        AutoRed = true;
      }
    if(x>60 & x<105){

    }
    }
    
    
    /*else if(60 < x){
      

    }
    else{
      left.set(0);
      right.set(0);
    } */

if (rwheelSpin > 50 & AutoRed == true) {
  if(rwheelSpin < 70){
    right.set(-0.3);
    left.set(0.3);
    m_collector.set(1);
  }
  else if(rwheelSpin > 71 & rwheelSpin < 151){
    left.set(-0.5);
      right.set(-0.5);
      m_collector.set(1);
    if(rwheelSpin > 146){
      rwheelSpin = 1000;
    }
  } else if(rwheelSpin > 978){
    right.set(0.3);
    left.set(-0.3);
    m_collector.set(1);
    if(rwheelSpin > 978){
      rwheelSpin = 2000;
    }
  }else if(rwheelSpin > 1999 & rwheelSpin < 2065){
    left.set(-0.5);
    right.set(-0.5);
    m_collector.set(1);
  } else if (x > 242 & x < 256) {
    right.set(-0.5);
    left.set(0.5);
    m_collector.set(1);
  } else if ( x > 256 & x < 301) {
    right.set(-0.5);
    left.set(-0.5);
    m_collector.set(1);
    
  }
  else{
    right.set(0);
    left.set(0);
  }

} else if (x > 60 && AutoRed == false) {

}








  
        
        
        }

      
       

      
       
    
  
  @Override
  public void teleopInit() {
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
    if(operateController.getRawButton(7)) {
      comp.start();
    }
     if(operateController.getRawButton(8)) {
       comp.stop();
     }
  }


    
}
