//
// Created by Hongyi "Mike" Fan on 2022-11-27
//

//AMBF Includes
#include <afFramework.h>
#include <algorithm>
#include <ambf_server/RosComBase.h>
#include <ambf_msgs/RigidBodyState.h>

//ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
//YAML Includes
#include <yaml-cpp/yaml.h>
//C++ Includes
#include <fstream>

using namespace ambf;

class AMBF_3DSlicer_Plugin: public afSimulatorPlugin {
    public:
        /*Required Functions For AMBF Simulator Plugin, MUST be implemented*/
        AMBF_3DSlicer_Plugin();
        virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
        virtual void graphicsUpdate() override;
        virtual void physicsUpdate(double dt) override;
        virtual void reset() override;
        virtual bool close() override;
        virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
        virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes) override;
        virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override;
        virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override;

        /*Public Methods*/
        bool volumetricDrillingInit();
        void toolCursorInit();
        void toolCursorPoseUpdate(cVector3d & pos);
        void volumetricDrillingServiceRoutine();                    //voxel removal algorithm, invoked in every physics update.
        void volumetricDrillingServiceRoutine_Graphics();           //voxel removal algorithm, invoked in every graphics update.

        /*Public Variables*/
        ros::NodeHandle* m_ROS_Node;
        ros::Publisher m_Dummy_Pub;
        tf::TransformBroadcaster* m_tf_broadcaster;
        std::vector<afVolumePtr> m_volumes; //This is a vector of all the volumes in the world

        cColorb m_zeroColor; //This is the color of the volume 
        cColorb m_boneColor;
        cColorb m_storedColor;

        afRigidBodyPtr m_drillRigidBody;
        cShapeSphere* m_burrMesh;
        afVolumePtr m_volumeObject;
        cVoxelObject* m_voxelObj;
        vector<cToolCursor*> m_toolCursorList;                              //List of tool cursors
        cGenericHapticDevicePtr m_hapticDevice;
        cMutex m_mutexVoxel;
        cCollisionAABBBox m_volumeUpdate;
        bool m_flagMarkVolumeForUpdate = false;

        

    private:
        void registerRobotMeshPaths();
};

AF_REGISTER_SIMULATOR_PLUGIN(AMBF_3DSlicer_Plugin);
