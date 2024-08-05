#include "AMBF_3DSlicer_Plugin.h"
#include <afConversions.h>
#include <iostream>

AMBF_3DSlicer_Plugin::AMBF_3DSlicer_Plugin() {
}

int AMBF_3DSlicer_Plugin::init(int argc, char** argv, const afWorldPtr a_afWorld) {
    std::cout << "AMBF 3DSlicer Plugin Initialized" << std::endl;
    //Get the world pointer
    m_worldPtr = a_afWorld;
    //Initialize ROS node
    m_ROS_Node = afROSNode::getNode();
    //Parse simulator attributes and publish robot link mesh paths to ROS parameter server
    registerRobotMeshPaths();
    //Setup tf broadcaster
    m_tf_broadcaster = new tf::TransformBroadcaster();
    //Initialize volumetric drilling
    volumetricDrillingInit();
    return 1;
}

void AMBF_3DSlicer_Plugin::registerRobotMeshPaths(){
    std::cout<<"Registering Robot Mesh Paths"<<std::endl;
    //Get all rigid bodies
    std::vector<afRigidBodyPtr> rigid_bodies = m_worldPtr->getRigidBodies();
    //Loop through all rigid bodies
    for(int i = 0; i < rigid_bodies.size(); i++){
        //Get the attributes of the rigid body
        afBaseObjectAttributes* objAttributes = rigid_bodies[i]->getAttributes();
        //Convert to rigid body attributes
        afRigidBodyAttributes* rigidBodyAttributes = (afRigidBodyAttributes*)(objAttributes);
        //Get the name of the rigid body
        std::string name = rigid_bodies[i]->getName();
        //Get the mesh path of the rigid body
        std::string mesh_path = rigidBodyAttributes->m_collisionAttribs.m_meshFilepath.c_str();
        //Check if the mesh path is valid by loading the mesh
        if( cMultiMesh().loadFromFile(mesh_path) == false){
            //If the mesh path is not valid, skip this rigid body
            continue;
        }
        //Get the parameter from the parameter server, if it exists
        std::string meshFilePathMap;
        if(!m_ROS_Node->getParam("/ambf/rigid_body_mesh_path_map",meshFilePathMap)){
            //If the parameter does not exist, set it to an empty string
            meshFilePathMap = "";
        }
        //Append the name, separeted by a colon, and the mesh path, separated by a semicolon
        meshFilePathMap += name + ":" + mesh_path + ";";
        //Set the parameter on the parameter server
        m_ROS_Node->setParam("/ambf/rigid_body_mesh_path_map", meshFilePathMap);
    }
}

void AMBF_3DSlicer_Plugin::graphicsUpdate(){ 
    //Update the volumetric drilling graphics
    volumetricDrillingServiceRoutine_Graphics();
}
void AMBF_3DSlicer_Plugin::physicsUpdate(double dt) {
    //Get all Rigidbody 
    std::vector<afRigidBodyPtr> rigid_bodies = m_worldPtr ->getRigidBodies();
    //Loop through all rigid bodies
    for (int i = 0; i < rigid_bodies.size(); i++) {
        //Get the name of the rigid body
        std::string name = rigid_bodies[i]->getName();
        //Get the pos and orientation of the rigid body
        chai3d::cVector3d pos = rigid_bodies[i]->getLocalPos();
        chai3d::cMatrix3d rot = rigid_bodies[i]->getLocalRot();
        //Construct a tf transform
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
        transform.setBasis(tf::Matrix3x3(rot(0, 0), rot(0, 1), rot(0, 2), rot(1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1), rot(2, 2)));
        //Broadcast the transform
        m_tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
    }

    //Volume Drilling Service
    volumetricDrillingServiceRoutine();

}

//This function initializes the volumetric drilling 
bool AMBF_3DSlicer_Plugin::volumetricDrillingInit() {
    //Find volume objects in world
    m_volumes = m_worldPtr->getVolumes();
    //Set voxel obj to first volume
    if (m_volumes.size() > 0) {
        m_voxelObj = m_volumes[0]->getInternalVolume();
    }
    //Set Color of voxels
    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);
    m_boneColor = cColorb(255, 249, 219, 255);
    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);

    /*Find drill in world map*/
    m_drillRigidBody = m_worldPtr->getRigidBody("drill_tip");  
    if (!m_drillRigidBody){
        /*If not in world, try finding it in Model map*/
        m_drillRigidBody = m_worldPtr->getRigidBody("drill_tip");
        /*Exit if fail to find it*/
        if (!m_drillRigidBody){
            cerr << "ERROR! FAILED TO FIND DRILL RIGID BODY NAMED " << "mastoidectomy_drill" << endl;
            exit(EXIT_FAILURE);
        }
    }
    m_burrMesh = new cShapeSphere(0.043); // 2mm by default with 1 AMBF unit = 0.049664 m
    m_burrMesh->setRadius(0.005);
    m_burrMesh->m_material->setBlack();
    m_burrMesh->m_material->setShininess(0);
    m_burrMesh->m_material->m_specular.set(0, 0, 0);
    m_burrMesh->setShowEnabled(true);
    m_drillRigidBody->addChildSceneObject(m_burrMesh, cTransform());
    m_worldPtr->addSceneObjectToWorld(m_burrMesh);

    toolCursorInit();
}

///
/// \brief This method contains the initialization of tool cursors in the af world
/// \return void
void AMBF_3DSlicer_Plugin::toolCursorInit(){
    m_toolCursorList.resize(1);                                 //TODO: currently only one tool cursor, may add more to 8 later
    for (int i = 0; i < m_toolCursorList.size() ; i++ ){
        m_toolCursorList[i] = new cToolCursor(m_worldPtr -> getChaiWorld());
        m_worldPtr->addSceneObjectToWorld(m_toolCursorList[i]);
        if(i == 0){
            //m_toolCursorList[i] -> setHapticDevice(m_hapticDevice);
            // map the physical workspace of the haptic device to a larger virtual workspace.
            m_toolCursorList[i]->setWorkspaceRadius(10.0);
            //m_toolCursorList[i]->setWaitForSmallForce(true);
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->setShowFrame(false);
            m_toolCursorList[i]-> setRadius(0.0043);
            m_toolCursorList[i]->m_name = "mastoidectomy_drill";
            //This method sets the display options of the goal and proxy spheres. If both spheres are enabled, a small line is drawn between both spheres.
            m_toolCursorList[i]->m_hapticPoint->setShow(false, false); 
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();
        }
        else{
            /*
            m_toolCursorList[i]->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
             m_toolCursorList[i]->setRadius(m_toolCursorRadius[i]);
            */
            m_toolCursorList[i]->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
            m_toolCursorList[i]->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        }
    }
    cVector3d toolPos = m_burrMesh->getLocalPos();
    toolCursorPoseUpdate(toolPos);
    for(int i = 0; i< m_toolCursorList.size(); i++){
        m_toolCursorList[i] -> initialize();
    }
}

///
/// \brief This method updates the position of the shaft tool cursors which eventually updates the position of the whole tool.
/// \return void
void AMBF_3DSlicer_Plugin::toolCursorPoseUpdate(cVector3d &pos){
    m_toolCursorList[0] -> setLocalPos(pos);
    m_toolCursorList[0] -> setDeviceGlobalPos(pos);
}

///
/// \brief This method contains the service routine for thevolumetric drilling algorithm. This method should be invoked in physics update
/// \return void
void AMBF_3DSlicer_Plugin::volumetricDrillingServiceRoutine(){
    //If no volume object is found, return
    if (!m_voxelObj){
        std::cout << "No volume object found" << std::endl;
        return;
    }
    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);
    //update tool cursor pos
    cVector3d toolPos = m_burrMesh->getLocalPos();
    toolCursorPoseUpdate(toolPos);
    //Voxel removing
    if (m_toolCursorList[0]->isInContact(m_voxelObj)){
        for (int ci = 0 ; ci < 3 ; ci++){
            // retrieve contact event
            cCollisionEvent* contact = m_toolCursorList[0]->m_hapticPoint->getCollisionEvent(ci);
            cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);
            m_voxelObj->m_texture->m_image->setVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_zeroColor);
            // update volume
            m_mutexVoxel.acquire();
            m_volumeUpdate.enclose(cVector3d(uint(orig.x()), uint(orig.y()), uint(orig.z())));
            m_mutexVoxel.release();
            
        }
        // mark voxel for update
        m_flagMarkVolumeForUpdate = true;
    }
    // compute interaction forces
    for(int i = 0 ; i < m_toolCursorList.size() ; i++){
        m_toolCursorList[i]->computeInteractionForces();
    }
}

///
/// \brief This method contains the additional service routine for thevolumetric drilling algorithm. This method should be invoked in graphics update
/// \return void
void AMBF_3DSlicer_Plugin::volumetricDrillingServiceRoutine_Graphics(){
    // update region of voxels to be updated
    if (m_flagMarkVolumeForUpdate)
    {
        m_mutexVoxel.acquire();
        cVector3d min = m_volumeUpdate.m_min;
        cVector3d max = m_volumeUpdate.m_max;
        m_volumeUpdate.setEmpty();
        m_mutexVoxel.release();
        ((cTexture3d*)m_voxelObj->m_texture.get())->markForPartialUpdate(min, max);
        m_flagMarkVolumeForUpdate = false;
    }
}

void AMBF_3DSlicer_Plugin::reset() {
    std::cout<<"Reset"<<std::endl;
}

bool AMBF_3DSlicer_Plugin::close() {
    std::cout<<"Close"<<std::endl;
    return true;
}

void AMBF_3DSlicer_Plugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods){

}

void AMBF_3DSlicer_Plugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes){

}

void AMBF_3DSlicer_Plugin::mousePosUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}

void AMBF_3DSlicer_Plugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}