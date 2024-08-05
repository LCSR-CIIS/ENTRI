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

// ROS1_Module Logic includes
#include "vtkSlicerROS1_ModuleLogic.h"

// MRML includes
#include <vtkMRMLScene.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLModelDisplayNode.h>
#include <vtkMRMLLinearTransformNode.h>
#include <vtkMRMLVolumeNode.h>
#include <vtkImageData.h>
#include <vtkImageMapToColors.h>
#include <vtkMRMLVolumeDisplayNode.h>
#include <vtkMRMLVolumeRenderingDisplayNode.h>


// STD includes
#include <cassert>

//ROS include
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

//Boost include
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

#define M_TO_MM 1000.0
#define RAD_TO_DEG 180.0/M_PI

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerROS1_ModuleLogic);

//----------------------------------------------------------------------------
vtkSlicerROS1_ModuleLogic::vtkSlicerROS1_ModuleLogic()
{
  //Create ROS Node
  int argc = 1;
  char** argv = new char*;
  const std::string name = GetClassName();
  argv[0] = new char[name.size() + 1];
  strcpy(argv[0], name.c_str());
  ros::init(argc, argv, "SlicerROSNode");
  slicerROSnodePtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));
  //ROS Publisher
  slicerPub = slicerROSnodePtr->advertise<std_msgs::Int64>("/slicerPub",1000);
  //ROS Subscriber 
  slicerSub = slicerROSnodePtr->subscribe("joint_states", 1000, &vtkSlicerROS1_ModuleLogic::slicerSubCallback, this);
  int counter = 0;
  //Load URDF from parameter server
  if (!m_urdfModel.initParam("robot_description"))
  {
    std::cout<<"Failed to parse urdf file"<<std::endl;
  }else{
    std::cout<<"URDF loaded"<<std::endl;
    std::cout<<"Robot name is: "<<m_urdfModel.getName()<<std::endl;
  }
  //Check if has AMBF in ROS Parameters
  if(slicerROSnodePtr->hasParam("/ambf/rigid_body_mesh_path_map")){
    std::cout<<"Connected to AMBF"<<std::endl;
    std::string AMBF_Mesh_Paths;
    slicerROSnodePtr->getParam("/ambf/rigid_body_mesh_path_map",AMBF_Mesh_Paths);
    LoadAMBFBodyMeshMap(AMBF_Mesh_Paths);
  }
  
  //Initialize tfListener
  m_tfListener_ptr = new tf::TransformListener();
}

//----------------------------------------------------------------------------
vtkSlicerROS1_ModuleLogic::~vtkSlicerROS1_ModuleLogic()
{
  //ROS Node Handle delete
  if(slicerROSnodePtr != NULL){
    //delete slicerROSnodePtr;
  }
  //Delete tfListener
  if(m_tfListener_ptr != NULL){
    delete m_tfListener_ptr;
  }
}

//----------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::SetMRMLSceneInternal(vtkMRMLScene * newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());
}

//-----------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);
  std::cout<<"Node Registered"<<std::endl;
  LoadURDFModelToScene();
  LoadAMBFModelToScene();
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic
::OnMRMLSceneNodeAdded(vtkMRMLNode* vtkNotUsed(node))
{
  std::cout<<"Node added"<<std::endl;
}

//---------------------------------------------------------------------------
void vtkSlicerROS1_ModuleLogic
::OnMRMLSceneNodeRemoved(vtkMRMLNode* vtkNotUsed(node))
{
}

void vtkSlicerROS1_ModuleLogic::Spin()
{
  //Spin ROS node
  if(ros::ok()){
    ros::spinOnce();
  }
  //update the model
  UpdateRobotModel();
  //Run Test Code function
  TestCode();

}

void vtkSlicerROS1_ModuleLogic::TestCode(){
  //Test Code for MRML Volume Node
  
  //Get all volume nodes
  vtkMRMLScene* scene = this->GetMRMLScene();
  vtkCollection* volumeNodes = scene->GetNodesByClass("vtkMRMLScalarVolumeNode");

  //Check if there is any volume node
  if(volumeNodes->GetNumberOfItems() == 0){
    std::cout<<"No volume node found"<<std::endl;
    return;
  }
  //Get the first volume node
  vtkMRMLVolumeNode* volumeNode = vtkMRMLVolumeNode::SafeDownCast(volumeNodes->GetItemAsObject(0));
  //std timestamp A in milliseconds
  std::chrono::milliseconds A = std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  );

  //Make all voxels random color
  vtkImageData* imageData = volumeNode->GetImageData();
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
      for(int k = 0; k < 5; k++){
        unsigned char* pixel = static_cast<unsigned char*>(imageData->GetScalarPointer(i,j,k));
        //Set random color
        pixel[0] = rand() % 255;
        pixel[1] = rand() % 255;
        pixel[2] = rand() % 255;
        //Set alpha to 255
        pixel[3] = 255;
      }
    }
  }
  //std timestamp B in milliseconds
  std::chrono::milliseconds B = std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  );
  
  //Update the volume
  volumeNode -> SetAndObserveImageData(imageData);
  volumeNode -> Modified();
  
  
  //std timestamp C in milliseconds
  std::chrono::milliseconds C = std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  );
  //Print out the time taken for each step
  std::cout<<"Time taken for setting pixel values: "<<B.count() - A.count()<<" ms"<<std::endl;
  std::cout<<"Time taken for updating volume: "<<C.count() - B.count()<<" ms"<<std::endl;
} 


void vtkSlicerROS1_ModuleLogic::slicerSubCallback(const std_msgs::Int64::ConstPtr& msg){
}

void vtkSlicerROS1_ModuleLogic::LoadURDFModelToScene(){
  //Get link from URDF model
  std::map<std::string, urdf::LinkSharedPtr > links = m_urdfModel.links_;
  std::map<std::string, urdf::LinkSharedPtr >::iterator it;
  //Create a model node for each link
  for (it = links.begin(); it != links.end(); it++)
  {
    std::cout<<"Link name: "<<it->first<<std::endl;
    //Convert Geometry pointer to Mesh pointer
    urdf::GeometrySharedPtr geometry;
    //Link offset XYZ and RPY
    double offsetXYZ[3];
    double offsetRPY[3];
    //Get link geometry and offset
    getLinkGeometryAndOffset(it->second, geometry, offsetXYZ, offsetRPY);
    //Check if geometry is NULL
    if (geometry == NULL){
      std::cout<<"No geometry found for link: "<<it->first<<std::endl;
      continue;
    }
    urdf::MeshSharedPtr mesh = boost::dynamic_pointer_cast<urdf::Mesh>(geometry);
    //Print mesh filename
    std::cout<<"Mesh filename: "<<mesh->filename<<std::endl;
    //Full path to mesh file
    std::string meshPath;
    //if contains "package://" prefix, get full path
    if(mesh->filename.find("package://") != std::string::npos){
      std::string packageName = mesh->filename.substr(10, mesh->filename.find("/", 10) - 10);//get package name
      std::string packagePath = ros::package::getPath(packageName);//get package path
      meshPath = packagePath + "/" + mesh->filename.substr(mesh->filename.find("/", 10) + 1);//get full path to mesh file
    }else{
      //if not, use the full path
      meshPath = mesh->filename;
    }
   
    //print mesh path
    std::cout<<"Mesh path: "<<meshPath<<std::endl;
    //Get polydata from mesh file
    vtkSmartPointer<vtkPolyData> polyData = ReadPolyData(meshPath.c_str());
    //Transform polydata with offset
    vtkSmartPointer<vtkTransform> offsetTransform = vtkSmartPointer<vtkTransform>::New();
    offsetTransform->Translate(offsetXYZ[0]*M_TO_MM, offsetXYZ[1]*M_TO_MM, offsetXYZ[2]*M_TO_MM);
    offsetTransform->RotateZ(offsetRPY[2]*RAD_TO_DEG);
    offsetTransform->RotateY(offsetRPY[1]*RAD_TO_DEG);
    offsetTransform->RotateX(offsetRPY[0]*RAD_TO_DEG);
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetTransform(offsetTransform);
    transformFilter->SetInputData(polyData);
    transformFilter->Update();
    polyData = transformFilter->GetOutput();
    //Set mesh to model node
    vtkNew<vtkMRMLModelNode> m;
    m->SetAndObservePolyData(polyData);
    //Set model node name
    m->SetName(it->first.c_str());
    //Set model node display node
    if (m->GetDisplayNodeID() == NULL){
      vtkNew<vtkMRMLModelDisplayNode> dnode;
      this->GetMRMLScene()->AddNode(dnode.GetPointer());
      dnode->SetName((it->first+"_display").c_str());
      m->SetAndObserveDisplayNodeID(dnode->GetID());
    }
    //Add model node and display node to scene
    if(this->GetMRMLScene() != NULL){
      this->GetMRMLScene()->AddNode(m.GetPointer());
      std::cout << "Model node added to scene" << std::endl;
    }else{
      std::cout<<"MRML Scene is NULL"<<std::endl;
    }
    //add model node to model node vector
    m_modelNodes.push_back(m.GetPointer());
  }
}

void vtkSlicerROS1_ModuleLogic::LoadAMBFModelToScene(){
  //Create model node for each rigidbody in ambf mesh map
  for (auto it = m_ambfBodyMeshMap.begin(); it != m_ambfBodyMeshMap.end(); it++)
  {
    std::cout<<"Link name: "<<it->first<<std::endl;
    //Get mesh path
    std::string meshPath = it->second;
    //print mesh path
    std::cout<<"Mesh path: "<<meshPath<<std::endl;
    //Get polydata from mesh file
    vtkSmartPointer<vtkPolyData> polyData = ReadPolyData(meshPath.c_str());
    //Set mesh to model node
    vtkNew<vtkMRMLModelNode> m;
    m->SetAndObservePolyData(polyData);
    //Set model node name
    m->SetName(it->first.c_str());
    //Set model node display node
    if (m->GetDisplayNodeID() == NULL){
      vtkNew<vtkMRMLModelDisplayNode> dnode;
      this->GetMRMLScene()->AddNode(dnode.GetPointer());
      dnode->SetName((it->first+"_display").c_str());
      m->SetAndObserveDisplayNodeID(dnode->GetID());
    }
    //Add model node and display node to scene
    if(this->GetMRMLScene() != NULL){
      this->GetMRMLScene()->AddNode(m.GetPointer());
      std::cout << "Model node added to scene" << std::endl;
    }else{
      std::cout<<"MRML Scene is NULL"<<std::endl;
    }
    //add model node to model node vector
    m_modelNodes.push_back(m.GetPointer());
  }

}

//Update model pose
void vtkSlicerROS1_ModuleLogic::UpdateRobotModel(){
  //Get model node from model node vector
  for(int i = 0; i < m_modelNodes.size(); i++){
    vtkMRMLModelNode* modelNode = m_modelNodes[i];
    //Get model node name
    std::string modelName = modelNode->GetName();
    //Get model node pose
    tf::StampedTransform transform;
    try{
      m_tfListener_ptr->lookupTransform(modelName, m_modelNodes[0]->GetName(), ros::Time(0), transform);
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    //Get 4x4 matrix from tf transform
    vtkSmartPointer<vtkMatrix4x4> matrix = GetMatrixFromTFtransform(transform);
    //Set model node pose
    if(modelNode -> GetParentTransformNode() == NULL){
      vtkNew<vtkMRMLLinearTransformNode> transformNode;
      transformNode->SetName((modelName+"_transform").c_str());
      this->GetMRMLScene()->AddNode(transformNode.GetPointer());
      modelNode->SetAndObserveTransformNodeID(transformNode->GetID());
    }else{
      vtkMRMLLinearTransformNode* transformNode = vtkMRMLLinearTransformNode::SafeDownCast(modelNode->GetParentTransformNode());
      transformNode->SetMatrixTransformFromParent(matrix);
    }
  }
}


//Function that converts STL, DAE or OBJ files to VTK PolyData 
vtkSmartPointer<vtkPolyData> vtkSlicerROS1_ModuleLogic::ReadPolyData(const char* fileName)
{
  vtkSmartPointer<vtkPolyData> polyData;
  std::string extension = vtksys::SystemTools::GetFilenameLastExtension(fileName);
  if (extension == ".stl" || extension == ".STL")
  {
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".obj" || extension == ".OBJ")
  {
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  else
  {
    vtkErrorMacro("File type not recognized: " << fileName << " only .stl, .STL, .obj, and .OBJ files are supported. Using dummy mesh instead.");
    //Create a sphere dummy mesh
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(0.0, 0.0, 0.0);
    sphereSource->SetRadius(50.0);
    sphereSource->Update();
    polyData = sphereSource->GetOutput();
  }
  //Convert units from meters to millimeters
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Scale(M_TO_MM, M_TO_MM, M_TO_MM);
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  transformFilter->SetInputData(polyData);
  transformFilter->Update();
  return transformFilter->GetOutput();
}

//Function to get vtkTransform matrix from tf transform
vtkSmartPointer<vtkMatrix4x4> vtkSlicerROS1_ModuleLogic::GetMatrixFromTFtransform(tf::StampedTransform transform){
  vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
  matrix->Identity();
  matrix->SetElement(0,0,transform.getBasis()[0][0]);
  matrix->SetElement(0,1,transform.getBasis()[0][1]);
  matrix->SetElement(0,2,transform.getBasis()[0][2]);
  matrix->SetElement(0,3,transform.getOrigin()[0] * M_TO_MM);
  matrix->SetElement(1,0,transform.getBasis()[1][0]);
  matrix->SetElement(1,1,transform.getBasis()[1][1]);
  matrix->SetElement(1,2,transform.getBasis()[1][2]);
  matrix->SetElement(1,3,transform.getOrigin()[1] * M_TO_MM);
  matrix->SetElement(2,0,transform.getBasis()[2][0]);
  matrix->SetElement(2,1,transform.getBasis()[2][1]);
  matrix->SetElement(2,2,transform.getBasis()[2][2]);
  matrix->SetElement(2,3,transform.getOrigin()[2] * M_TO_MM);
  return matrix;
}

void vtkSlicerROS1_ModuleLogic::getLinkGeometryAndOffset(urdf::LinkSharedPtr & link, urdf::GeometrySharedPtr & geometry, double* offsetXYZ, double* offsetRPY){
  if(link->collision != NULL ){
      geometry = link->collision->geometry;
      offsetXYZ[0] = link->collision->origin.position.x;
      offsetXYZ[1] = link->collision->origin.position.y;
      offsetXYZ[2] = link->collision->origin.position.z;
      link->collision->origin.rotation.getRPY(offsetRPY[0], offsetRPY[1], offsetRPY[2]);
    }else{
      //If no collision geometry, use visual geometry
      if(link->visual != NULL){
        geometry = link->visual->geometry;
        offsetXYZ[0] = link->visual->origin.position.x;
        offsetXYZ[1] = link->visual->origin.position.y;
        offsetXYZ[2] = link->visual->origin.position.z;
        link->visual->origin.rotation.getRPY(offsetRPY[0], offsetRPY[1], offsetRPY[2]);
      }else{
        geometry = NULL;
      }
    }
}

void vtkSlicerROS1_ModuleLogic::LoadAMBFBodyMeshMap(std::string ambfBodyMeshMapParam){
  //Different rigidbodies are separated by semi-colons, and the name of the rigidbody and the path to the mesh are separated by a colon
  //Boost Split the string by semi-colons
  std::vector<std::string> ambfBodyMeshMapVector;
  boost::split(ambfBodyMeshMapVector, ambfBodyMeshMapParam, boost::is_any_of(";"));
  //Loop through the vector and split each element by the colon
  for(int i = 0; i < ambfBodyMeshMapVector.size(); i++){
    std::vector<std::string> ambfBodyMeshMapElementVector;
    boost::split(ambfBodyMeshMapElementVector, ambfBodyMeshMapVector[i], boost::is_any_of(":"));
    //If the element vector has two elements, then the first element is the name of the rigidbody and the second element is the path to the mesh
    if(ambfBodyMeshMapElementVector.size() == 2){
      this->m_ambfBodyMeshMap[ambfBodyMeshMapElementVector[0]] = ambfBodyMeshMapElementVector[1];
    }
  }
}

