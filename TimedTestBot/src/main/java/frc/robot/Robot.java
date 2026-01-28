// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final XboxController xbox = new XboxController(0);
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final SparkMax m_shooterMaster = new SparkMax(30, MotorType.kBrushless);
    
    //private final SparkMax m_shooterSlave = new SparkMax(31, MotorType.kBrushless);
    private final SparkClosedLoopController m_shooterPID = m_shooterMaster.getClosedLoopController();
    private final RelativeEncoder m_shooterEncoder = m_shooterMaster.getEncoder();
    // Default PID/FF values (starting points - tune on the robot)
    private double kP = 0.0004;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kff_ks = 0.05;
    private double kff_kv = 1.0 / 5700.0;
    private double kMinOutput = -1.0;
    private double kMaxOutput = 1.0;
    private final AnalogInput m_analogInput = new AnalogInput(0);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    SparkMaxConfig masterConfig = new SparkMaxConfig();
    masterConfig.inverted(true);
    FeedForwardConfig ffcfg = new FeedForwardConfig();
    ffcfg.kS(kff_ks);
    ffcfg.kV(kff_kv);
;    //ffcfg.kS(kDefaultPeriod)
    //SparkMaxConfig slaveConfig = new SparkMaxConfig();

    masterConfig.closedLoop.pid(kP, kI, kD);
    masterConfig.closedLoop.feedForward.apply(ffcfg);
    

    m_shooterMaster.configure(masterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  // Configure closed-loop (velocity) PID controller with reasonable defaults.
  // These values are starting points only; tune on the real robot using the dashboard.
  // Publish tuning values to SmartDashboard so they can be adjusted live.
  SmartDashboard.putNumber("Shooter P", kP);
  SmartDashboard.putNumber("Shooter I", kI);
  SmartDashboard.putNumber("Shooter D", kD);
  SmartDashboard.putNumber("Shooter MinOutput", kMinOutput);
  SmartDashboard.putNumber("Shooter MaxOutput", kMaxOutput);
  SmartDashboard.putNumber("Shooter Target RPM", 0.0);

    // Slave is just set to follow
    //slaveConfig.follow(30, true);
    //m_shooterSlave.configure(slaveConfig, ResetMode.kResetSafeParameters,
    //    PersistMode.kPersistParameters);

}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_shooterMaster.set(0.0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    double potInput = m_analogInput.getVoltage();
    double targetRPM = potInput / 5.0 * 5700.0; // Scale 0-5V to 0-5700 RPM
    // Need to adjust for deadzone of 15% for joystick input
    double leftY = xbox.getLeftY();
    if (Math.abs(leftY) < 0.15)
    {
      leftY = 0.0;
    }
    //m_shooterMaster.set(leftY);

    if (xbox.getAButton())
    {
      // Set the m_shooterMaster target velocity
      m_shooterPID.setSetpoint(targetRPM, ControlType.kVelocity);
    }
    else
    {
      if (leftY != 0.0)
      {
        m_shooterMaster.set(leftY);
      }
      else
      {
        m_shooterMaster.stopMotor();
      }
    }
    SmartDashboard.putNumber("AnalogIn", potInput);
    SmartDashboard.putNumber("TargetRPM", targetRPM);
    SmartDashboard.putNumber("XboxLeftY", leftY);
    SmartDashboard.putNumber("MotorRPM", m_shooterMaster.getEncoder().getVelocity());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() 
  {
    m_shooterMaster.set(0.0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
