package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

public final class ShooterCalc {

        private static final InterpolatingDoubleTreeMap hoodMap = Constants.ShooterConstants.kShotHoodAngleMap;
        private static final InterpolatingDoubleTreeMap flywheelMap = Constants.ShooterConstants.kShotFlywheelSpeedMap;
        private static final InterpolatingDoubleTreeMap tofMap = Constants.ShooterConstants.kTimeOfFlightMap;

        private static final LinearFilter hoodFilter = LinearFilter
                        .movingAverage((int) (0.1 / Constants.loopPeriodSecs));
        private static final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        private static final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

        private static final Translation2d HUB_POS = AllianceFlipUtil
                        .apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
        private static final Pose2d ZERO_POSE = new Pose2d();

        /** Cached result */
        private static ShootingParameters cachedParams = new ShootingParameters(false, 0.0, 0.0, ZERO_POSE);

        /** Ensures we only compute once per loop */
        private static double lastTimestamp = -1.0;

        public record ShootingParameters(
                        boolean isValid,
                        double hoodAngle,
                        double flywheelSpeed,
                        Pose2d target) {
        }

        /** Call ONCE per robot loop (periodic / superstructure update) */
        public static void update() {
                double now = Timer.getFPGATimestamp();
                if (now == lastTimestamp) {
                        return; // already computed this loop
                }
                lastTimestamp = now;

                var drive = Superstructure.getInstance().getDrivebase();
                Pose2d robotPose = drive.getPose();

                Translation2d robotTranslation = robotPose.getTranslation();
                double distance = robotTranslation.getDistance(HUB_POS);

                if (distance <= 0.0 ||
                                distance > Constants.ShooterConstants.kMaxShootingDist ||
                                !Vision.getInstance().isInAllianceZone()) {

                        cachedParams = new ShootingParameters(false, 0.0, 0.0, ZERO_POSE);
                        return;
                }

                double timeOfFlight = tofMap.get(distance);

                ChassisSpeeds speeds = drive.getFieldVelocity();

                double smoothVx = xFilter.calculate(speeds.vxMetersPerSecond);
                double smoothVy = yFilter.calculate(speeds.vyMetersPerSecond);

                Translation2d virtualTarget = new Translation2d(
                                HUB_POS.getX() - smoothVx * timeOfFlight,
                                HUB_POS.getY() - smoothVy * timeOfFlight);

                double hoodAngle = hoodFilter.calculate(hoodMap.get(distance));

                Translation2d raw = HUB_POS.minus(robotTranslation);
                double invNorm = raw.getNorm() > 1e-6 ? 1.0 / raw.getNorm() : 0.0;
                Translation2d shotDir = raw.times(invNorm);

                double velocityTowardGoal = smoothVx * shotDir.getX() +
                                smoothVy * shotDir.getY();

                double baseRPM = flywheelMap.get(distance);

                double radialComp = Constants.ShooterConstants.kRadialRPMComp * velocityTowardGoal;

                radialComp = MathUtil.clamp(
                                radialComp,
                                -Constants.ShooterConstants.kRadialRPMComp,
                                Constants.ShooterConstants.kRadialRPMComp);

                // ignore small velocity
                if (Math.abs(velocityTowardGoal) < Constants.ShooterConstants.kSidewaysVelocityDeadband) {
                        radialComp = 0.0;
                }

                if (Math.hypot(smoothVx, smoothVy) < Constants.ShooterConstants.kMeaningfullVelocity) {
                        radialComp = 0.0;
                }

                double flywheelSpeed = baseRPM + radialComp;

                cachedParams = new ShootingParameters(
                                true,
                                hoodAngle,
                                flywheelSpeed,
                                new Pose2d(virtualTarget, Rotation2d.kZero));
        }

        /** Zero-cost getter */
        public static ShootingParameters getParameters() {
                return cachedParams;
        }

        public static void resetHoodFilter() {
                hoodFilter.reset();
        }
}
