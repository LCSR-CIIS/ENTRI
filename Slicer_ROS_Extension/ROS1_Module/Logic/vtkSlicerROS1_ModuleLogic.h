/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

==============================================================================*/

// .NAME vtkSlicerROS1_ModuleLogic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerROS1_ModuleLogic_h
#define __vtkSlicerROS1_ModuleLogic_h

// Slicer includes
#include "vtkSlicerModuleLogic.h"

// MRML includes
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>

// STD includes
#include <cstdlib>

#include "vtkSlicerROS1_ModuleModuleLogicExport.h"

//ROS includes
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <urdf/model.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/package.h>


/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_ROS1_MODULE_MODULE_LOGIC_EXPORT vtkSlicerROS1_ModuleLogic :
  public vtkSlicerModuleLogic
{
public:

  static vtkSlicerROS1_ModuleLogic *New();
  vtkTypeMacro(vtkSlicerROS1_ModuleLogic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  void Spin();
  
  //tfListener
  tf::TransformListener* m_tfListener_ptr;
  //ROS Node Pointer
  ros::NodeHandlePtr slicerROSnodePtr;
  //ROS Publisher
  ros::Publisher slicerPub;
  //ROS Subscriber
  ros::Subscriber slicerSub;
  //ROS Subscriber Callback function
  void slicerSubCallback(const std_msgs::Int64::ConstPtr& msg);
  //URDF model
  urdf::Model m_urdfModel;
  //Map for storing AMBF body names and their mesh file names
  std::map<std::string, std::string> m_ambfBodyMeshMap;
  //MRML Model Node vector
  std::vector<vtkMRMLModelNode*> m_modelNodes;
  //Function to load URDF model to scene
  void LoadURDFModelToScene();
  //Function to load AMBF model to scene
  void LoadAMBFModelToScene();
  //Function to update URDF model in scene
  void UpdateRobotModel();
  //Function that converts STL, DAE or OBJ files to VTK PolyData
  vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileName);
  //Function to get vtk transform matrix from tf transform
  vtkSmartPointer<vtkMatrix4x4>GetMatrixFromTFtransform(tf::StampedTransform transform);
  //Function to get link geometry and link offset
  void getLinkGeometryAndOffset(urdf::LinkSharedPtr & link, urdf::GeometrySharedPtr & geometry, double* offsetXYZ, double* offsetRPY);
  //Function to Load AMBF rigid body and Mesh map
  void LoadAMBFBodyMeshMap(std::string ambfBodyMeshMapParam);
  
  //Function to test code
  void TestCode();


protected:
  vtkSlicerROS1_ModuleLogic();
  ~vtkSlicerROS1_ModuleLogic() override;

  void SetMRMLSceneInternal(vtkMRMLScene* newScene) override;
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  void RegisterNodes() override;
  void UpdateFromMRMLScene() override;
  void OnMRMLSceneNodeAdded(vtkMRMLNode* node) override;
  void OnMRMLSceneNodeRemoved(vtkMRMLNode* node) override;
private:

  vtkSlicerROS1_ModuleLogic(const vtkSlicerROS1_ModuleLogic&); // Not implemented
  void operator=(const vtkSlicerROS1_ModuleLogic&); // Not implemented

  //Testcode Flag
  bool m_testCodeFlag = false;
};

#endif
