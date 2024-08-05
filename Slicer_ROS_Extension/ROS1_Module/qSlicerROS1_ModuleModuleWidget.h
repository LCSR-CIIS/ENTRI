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

#ifndef __qSlicerROS1_ModuleModuleWidget_h
#define __qSlicerROS1_ModuleModuleWidget_h

// Slicer includes
#include "qSlicerAbstractModuleWidget.h"

#include "qSlicerROS1_ModuleModuleExport.h"

class qSlicerROS1_ModuleModuleWidgetPrivate;
class vtkMRMLNode;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_ROS1_MODULE_EXPORT qSlicerROS1_ModuleModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

public:

  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerROS1_ModuleModuleWidget(QWidget *parent=0);
  virtual ~qSlicerROS1_ModuleModuleWidget();

public slots:


protected:
  QScopedPointer<qSlicerROS1_ModuleModuleWidgetPrivate> d_ptr;

  void setup() override;
  QTimer* mTimer;
  bool timerOff = false;

protected slots:
  void onTimerTimeOut(void);
  
private:
  Q_DECLARE_PRIVATE(qSlicerROS1_ModuleModuleWidget);
  Q_DISABLE_COPY(qSlicerROS1_ModuleModuleWidget);


};

#endif
