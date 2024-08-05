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

#ifndef __qSlicerROS1_ModuleFooBarWidget_h
#define __qSlicerROS1_ModuleFooBarWidget_h

// Qt includes
#include <QWidget>

// FooBar Widgets includes
#include "qSlicerROS1_ModuleModuleWidgetsExport.h"

class qSlicerROS1_ModuleFooBarWidgetPrivate;

/// \ingroup Slicer_QtModules_ROS1_Module
class Q_SLICER_MODULE_ROS1_MODULE_WIDGETS_EXPORT qSlicerROS1_ModuleFooBarWidget
  : public QWidget
{
  Q_OBJECT
public:
  typedef QWidget Superclass;
  qSlicerROS1_ModuleFooBarWidget(QWidget *parent=0);
  ~qSlicerROS1_ModuleFooBarWidget() override;

protected slots:

protected:
  QScopedPointer<qSlicerROS1_ModuleFooBarWidgetPrivate> d_ptr;

private:
  Q_DECLARE_PRIVATE(qSlicerROS1_ModuleFooBarWidget);
  Q_DISABLE_COPY(qSlicerROS1_ModuleFooBarWidget);
};

#endif
