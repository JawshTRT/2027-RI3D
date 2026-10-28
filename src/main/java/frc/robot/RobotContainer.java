package frc.robot;

import static edu.wpi.first.units.Units.MilliOhms;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.SpektrumDriveController;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
//import frc.robot.subsystems.vision.LimelightVisionSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.simulation.BatterySim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  final DriveController driveController = new SpektrumDriveController(0);
  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();

  private final SwerveInputStream driveStream = swerve.getAngularVelocityStream(driveController::getDriveX,
                                                                                 driveController::getDriveY,
                                                                                 driveController::getRotation)
                                                       .withDeadband(0.05)
                                                       .withAllianceRelativeControl();
  public RobotContainer()
  {
    configureBatterySim();
    configureBindings();
  }

  /**
   * Configure the simulated battery's state-of-charge curve and discharge behavior.
   */
  private void configureBatterySim()
  {
    InterpolatingDoubleTreeMap wornBatteryCurve = new InterpolatingDoubleTreeMap();
    wornBatteryCurve.put(0.00, 6.0);
    wornBatteryCurve.put(0.05, 6.5);
    wornBatteryCurve.put(0.10, 9.5);
    wornBatteryCurve.put(0.20, 10.2);
    wornBatteryCurve.put(0.40, 10.6);
    wornBatteryCurve.put(0.60, 10.9);
    wornBatteryCurve.put(0.80, 11.2);
    wornBatteryCurve.put(0.90, 11.4);
    wornBatteryCurve.put(1.00, 11.6);
    BatterySim.replaceSOCInterpolation(wornBatteryCurve);
    BatterySim.enableDischarge(18, Volts.of(11), MilliOhms.of(20));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    swerve.setDefaultCommand(swerve.drive(driveStream));
    driveController.reset().onTrue(swerve.zeroGyro());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return null;
  }

}
