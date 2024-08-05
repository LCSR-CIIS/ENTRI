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
#include <vtkSlicerROS1_ModuleLogic.h>

// ROS1_Module includes
#include "qSlicerROS1_ModuleModule.h"
#include "qSlicerROS1_ModuleModuleWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerROS1_ModuleModulePrivate
{
public:
  qSlicerROS1_ModuleModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerROS1_ModuleModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleModulePrivate::qSlicerROS1_ModuleModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerROS1_ModuleModule methods

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleModule::qSlicerROS1_ModuleModule(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerROS1_ModuleModulePrivate)
{
}

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleModule::~qSlicerROS1_ModuleModule()
{
}

//-----------------------------------------------------------------------------
QString qSlicerROS1_ModuleModule::helpText() const
{
  return "This is a loadable module that can be bundled in an extension";
}

//-----------------------------------------------------------------------------
QString qSlicerROS1_ModuleModule::acknowledgementText() const
{
  return "This work was partially funded by NIH grant NXNNXXNNNNNN-NNXN";
}

//-----------------------------------------------------------------------------
QStringList qSlicerROS1_ModuleModule::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("Mike Fan (Johns Hopkins University)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerROS1_ModuleModule::icon() const
{
  return QIcon(":/Icons/ROS1_Module.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerROS1_ModuleModule::categories() const
{
  return QStringList() << "Examples";
}

//-----------------------------------------------------------------------------
QStringList qSlicerROS1_ModuleModule::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerROS1_ModuleModule::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerROS1_ModuleModule
::createWidgetRepresentation()
{
  return new qSlicerROS1_ModuleModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerROS1_ModuleModule::createLogic()
{
  return vtkSlicerROS1_ModuleLogic::New();
}
