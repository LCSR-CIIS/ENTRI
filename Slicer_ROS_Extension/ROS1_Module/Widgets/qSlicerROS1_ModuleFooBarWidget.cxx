/*==============================================================================

  Program: 3D Slicer

  Copyright (c) Kitware Inc.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc.
  and was partially funded by NIH grant 3P41RR013218-12S1

==============================================================================*/

// FooBar Widgets includes
#include "qSlicerROS1_ModuleFooBarWidget.h"
#include "ui_qSlicerROS1_ModuleFooBarWidget.h"

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ROS1_Module
class qSlicerROS1_ModuleFooBarWidgetPrivate
  : public Ui_qSlicerROS1_ModuleFooBarWidget
{
  Q_DECLARE_PUBLIC(qSlicerROS1_ModuleFooBarWidget);
protected:
  qSlicerROS1_ModuleFooBarWidget* const q_ptr;

public:
  qSlicerROS1_ModuleFooBarWidgetPrivate(
    qSlicerROS1_ModuleFooBarWidget& object);
  virtual void setupUi(qSlicerROS1_ModuleFooBarWidget*);
};

// --------------------------------------------------------------------------
qSlicerROS1_ModuleFooBarWidgetPrivate
::qSlicerROS1_ModuleFooBarWidgetPrivate(
  qSlicerROS1_ModuleFooBarWidget& object)
  : q_ptr(&object)
{
}

// --------------------------------------------------------------------------
void qSlicerROS1_ModuleFooBarWidgetPrivate
::setupUi(qSlicerROS1_ModuleFooBarWidget* widget)
{
  this->Ui_qSlicerROS1_ModuleFooBarWidget::setupUi(widget);
}

//-----------------------------------------------------------------------------
// qSlicerROS1_ModuleFooBarWidget methods

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleFooBarWidget
::qSlicerROS1_ModuleFooBarWidget(QWidget* parentWidget)
  : Superclass( parentWidget )
  , d_ptr( new qSlicerROS1_ModuleFooBarWidgetPrivate(*this) )
{
  Q_D(qSlicerROS1_ModuleFooBarWidget);
  d->setupUi(this);
}

//-----------------------------------------------------------------------------
qSlicerROS1_ModuleFooBarWidget
::~qSlicerROS1_ModuleFooBarWidget()
{
}
