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

// Qt includes
#include <QDebug>
#include <QTimer>


// Slicer includes
#include "qSlicerROS1_ModuleModuleWidget.h"
#include "ui_qSlicerROS1_ModuleModuleWidget.h"
#include "qSlicerApplication.h"
// reference to Logic
#include "vtkSlicerROS1_ModuleLogic.h"

//Slicer includes
#include "vtkMRMLModelDisplayNode.h"
#include "vtkMRMLMarkupsDisplayNode.h"
#include "vtkMRMLDisplayNode.h"
#include "vtkMRMLModelNode.h"
#include "vtkMRMLMarkupsFiducialNode.h"
#include "vtkMRMLInteractionNode.h"
#include "vtkMRMLScene.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerROS1_ModuleModuleWidgetPrivate: public Ui_qSlicerROS1_ModuleModuleWidget
{
public:
  qSlicerROS1_ModuleModuleWidgetPrivate(); 
  vtkSlicerROS1_ModuleLogic* logic() const;
};

//-----------------------------------------------------------------------------
// qSlicerROS1_ModuleModuleWidgetPrivate methods

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleModuleWidgetPrivate::qSlicerROS1_ModuleModuleWidgetPrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerROS1_ModuleModuleWidget methods

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleModuleWidget::qSlicerROS1_ModuleModuleWidget(QWidget* _parent)
  : Superclass( _parent )
  , d_ptr( new qSlicerROS1_ModuleModuleWidgetPrivate )
{
  //Initialize qTimer
  this->mTimer = new QTimer();
  mTimer->setSingleShot(false);
  mTimer->setInterval(5); // 5ms, 200Hz
  mTimer->start();
}

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleModuleWidget::~qSlicerROS1_ModuleModuleWidget()
{
  mTimer->stop();
  delete this->mTimer;
}

//-----------------------------------------------------------------------------
void qSlicerROS1_ModuleModuleWidget::setup()
{
  Q_D(qSlicerROS1_ModuleModuleWidget);
  d->setupUi(this);
  this->Superclass::setup();

  // Set up timer connection
  connect(mTimer, SIGNAL(timeout()), this, SLOT(onTimerTimeOut()));
}


void qSlicerROS1_ModuleModuleWidget::onTimerTimeOut()
{
  Q_D(qSlicerROS1_ModuleModuleWidget);
  this->Superclass::setup();

  vtkSlicerROS1_ModuleLogic* logic = vtkSlicerROS1_ModuleLogic::SafeDownCast(this->logic());
  if (!logic) {
    qWarning() << Q_FUNC_INFO << "failed: Invalid Slicer ROS Module Logic";
    return;
  }
  logic -> Spin();
}