#include <vision/LimelightHelpers.h>
#include <frc/geometry/Pose2d.h>

inline std::vector<LimelightHelpers::RawFiducial> getRawFiducials(std::string LL_id){ //LL_id stands for limelight_id changed from last years code which was limelight_id
    LimelightHelpers::PoseEstimate estPostData=LimelightHelpers::getBotPoseEstimate(LL_id,"botpose",false);
    return estPostData.rawFiducials;
}

inline frc::Pose2d getPose2d(std::string camera){
    return LimelightHelpers::getBotPose2d(camera);
}


