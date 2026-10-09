package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.gyros.Gyro;
import frc.robot.gyros.NavxGyro;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.parser.SwerveParser;
import swervelib.parser.SwerveParser.SwerveDriveDevices;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SwerveDriveTelemetryConfig;

public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveDrive drive;
    private Gyro gyro;

    public SwerveDriveSubsystem() {
        SmartDashboard.putData(this);
        var cfg = new SwerveDriveConfig()
            .withSubsystem(this)
            .withGyro(() -> Degrees.of(gyro.getRealGyroAngle()))
            .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
            .withTranslationController(new PIDController(4, 0, 0))
            .withRotationController(new PIDController(1, 0, 0))
            .withTelemetry(
                "swerve",
                new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH)
            );

        SwerveParser.parse(
            new File(Filesystem.getDeployDirectory(), "swerve/base")
        );
        SwerveDriveDevices devices = SwerveParser.createSwerveDriveDevices(cfg);
        drive = devices.swerveDrive();
        gyro = new NavxGyro();
    }

    /**
     * Drive to the given point on the field using YAMS' {@link SwerveDrive#driveToPose(Pose2d)}.
     *
     * @param point {@link Pose2d} to drive to. Field relative, blue-origin where 0deg is facing towards RED alliance.
     * @return {@link Command} that drives to the given point.
     */
    public Command driveToPointYAMS(Pose2d point) {
        return drive.driveToPose(point);
    }

    /**
     * Zero the gyro, resetting the robot's heading to face away from the driver station (0deg, red alliance side).
     *
     * @return {@link Command} that zeroes the gyro.
     */
    public Command zeroGyro() {
        return Commands.runOnce(drive::zeroGyro, this).withName("Zero Gyro");
    }

    public SwerveInputStream getAngularVelocityStream(
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rot
    ) {
        return new SwerveInputStream(drive, x, y, rot);
    }

    /**
     * Get the current heading of the robot, as reported by odometry.
     *
     * @return {@link Rotation2d} of the robot's heading.
     */
    public Rotation2d getHeading() {
        return new Rotation2d(drive.getGyroAngle());
    }

    /**
     * Get the gyro's full 3 axis orientation (roll, pitch, and yaw) as a {@link Rotation3d}.
     *
     * @return {@link Rotation3d} of the gyro.
     */
    public Rotation3d getGyroRotation3d() {
        if (RobotBase.isSimulation()) {
            // Only required if you cant simulate the gyro's orientation.'
            return new Rotation3d(
                0,
                0,
                drive.getSimPose().getRotation().getRadians()
            );
        }
        return new Rotation3d(gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
    }

    /**
     * Gets the measured pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose.
     */
    public Pose2d getPose() {
        return drive.getPose();
    }

    /**
     * Reset the pose estimator (and, in simulation, the ground truth pose) to the given pose.
     *
     * @param pose Pose to reset to. Field relative, blue-origin.
     */
    public void resetOdometry(Pose2d pose) {
        drive.resetOdometry(pose);
    }

    /**
     * Get the {@link Field2d} used to display the robot's pose, so callers (e.g. vision subsystems) can publish
     * additional {@link edu.wpi.first.wpilibj.smartdashboard.FieldObject2d}s onto the same field widget instead of
     * creating their own.
     *
     * @return {@link Field2d} of the drive.
     */
    public Field2d getField2d() {
        return drive.getField2d();
    }

    /**
     * Fuse a vision-derived pose measurement, e.g. from {@link frc.robot.subsystems.vision.LimelightVisionSubsystem}, into the
     * drive's pose estimator.
     *
     * @param visionPose        Vision-measured {@link Pose2d}, field relative, blue-origin.
     * @param timestampSeconds  Timestamp the measurement was taken at, matching {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.
     */
    public void addVisionMeasurement(
        Pose2d visionPose,
        double timestampSeconds
    ) {
        drive.addVisionMeasurement(visionPose, timestampSeconds);
    }

    public Command drive(SwerveInputStream stream) {
        return drive.drive(() ->
            ChassisSpeeds.fromFieldRelativeSpeeds(
                stream.get(),
                new Rotation2d(drive.getGyroAngle())
            )
        );
    }

    /**
     * Create a {@link Command} that runs a full SysId characterization routine (quasistatic and dynamic, forward and
     * reverse) on a single swerve module's drive motor. The module's azimuth is held pointed straight ahead for the
     * duration of the test so only the drive motor is characterized.
     *
     * @param moduleName Name of the module to test, e.g. "frontleft", "frontright", "backleft", or "backright".
     * @return {@link Command} that runs the full SysId routine on the given module.
     */
    public Command sysIdModule(String moduleName) {
        SwerveModule module = drive.getModule(moduleName).orElseThrow();
        SmartMotorController azimuthMotor = module.getAzimuthMotorController();

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(7),
                Seconds.of(10)
            ),
            new SysIdRoutine.Mechanism(
                azimuthMotor::setVoltage,
                log ->
                    log
                        .motor(moduleName + "-azimuth")
                        .voltage(azimuthMotor.getVoltage())
                        .angularPosition(azimuthMotor.getMechanismPosition())
                        .angularVelocity(azimuthMotor.getMechanismVelocity()),
                this,
                moduleName + "-azimuth"
            )
        );

        return Commands.runOnce(() ->
            azimuthMotor.setPosition(Rotation2d.kZero.getMeasure())
        )
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1))
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.waitSeconds(1))
            .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1))
            .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
            .withName("SysId " + moduleName + " Azimuth");
    }

    public void periodic() {
        drive.updateTelemetry();
    }

    public void simulationPeriodic() {
        drive.simIterate();
    }
}
