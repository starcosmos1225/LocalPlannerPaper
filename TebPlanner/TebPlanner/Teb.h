namespace TebPlanner{
class Teb
{
public:
    /**
     * @brief Teb:Default constructor
     */
    Teb();
    /**
     * @brief ~Teb:Default destructor
     */
    ~Teb();
    /**
     * @brief initilize:init the Teb
     */
    void initilize();
    /**
     * @brief insertPose: insert a pose in teb_pose_containner
     * @param pose: the insert pose
     */
    void insertPose(Pose2D pose);
    /**
     * @brief insertTimeDiff: insert a timeDiff in teb_timediff_containner
     * @param timediff: the time interval between points
     */
    void insertTimeDiff(double timediff);
    /**
     * @brief deletePose: delete the pose
     * @param pose:the pose need to be deleted
     * @return True if delete success, otherwise false
     */
    bool deletePose(Pose2D pose);
    /**
     * @brief deleteTimeDiff: delete the timediff
     * @param timediff: the timediff need to be deleted
     * @return True if delete success, otherwise false
     */
    bool deleteTimeDiff(double timediff);
    /**
     * @brief poseTop return the top element in point containner
     * @return
     */
    Pose2D poseTop();
    /**
     * @brief timeDiffTop return the top element in timediff containner
     * @return
     */
    double timeDiffTop();
    /**
     * @brief clearPose clear the point containner
     */
    void clearPose();
    /**
     * @brief clearTimeDiff clear the timeddf containner
     */
    void clearTimeDiff();
    /**
     * @brief Point:get teb point[index]
     * @param index
     * @return the pointer of point[index]
     */
    Pose2D* Point(int index)
    {
        ROS_ASSERT(index<teb_pose_containner_.size());
        return teb_pose_containner_.at(index);
    }
    /**
     * @brief timeDiff:get teb timeDiff[index]
     * @param index
     * @return the pointer of timeDiff[index]
     */
    double* timeDiff(int index)
    {
        ROS_ASSERT(index<teb_timediff_containner_.size());
        return teb_timediff_containner_.at(index);
    }

protected://!<protected variable
    std::vector<Pose2D* > teb_pose_containner_;//!<Teb local point containner;
    std::vector<double* > teb_timediff_containner_;//!<Teb time interval between point_i and point_(i+1)
};
}
