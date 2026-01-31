package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LinesVertical;
import swervelib.SwerveDrive;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vision {
    private static Vision _instance;    
    public static Vision getInstance() {
        if (_instance == null) {
            _instance = new Vision();
        }
        return _instance;
    }
    
    private final String _limelightName;
    private Pose2d currentPosition;

    private Vision() {
        _limelightName = Constants.VisionConstants.kLimelightName;
        currentPosition = new Pose2d();
    }

    public boolean updatePoseEstimation(SwerveDrive swerveDrive) {
        double robotYaw = swerveDrive.getYaw().getDegrees();
        LimelightHelpers.SetRobotOrientation(_limelightName, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    
        PoseEstimate est = null;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);
        } else {
            est = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(_limelightName);
        }

        currentPosition = swerveDrive.getPose();

        if (est == null) return false;
        if (est.tagCount < 1) return false;
        if (est.pose == null) return false;

        swerveDrive.addVisionMeasurement(est.pose,est.timestampSeconds);
        currentPosition = swerveDrive.getPose();
        
        return true;
    }

    public Pose2d getPosition() {return currentPosition;}

    public boolean isInAllianceZone() {
        var x = AllianceFlipUtil.applyX(currentPosition.getX());
        return x < LinesVertical.allianceZone;
    }

    public Translation2d getDistanceToHub() {
        return getPosition().getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint).toTranslation2d());
    }
}
