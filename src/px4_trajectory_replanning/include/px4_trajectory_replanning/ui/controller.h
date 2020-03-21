#ifndef UI_CONTROLLER_H
#define UI_CONTROLLER_H

#include <QMainWindow>
#include <QWidget>
#include <px4_trajectory_replanning/ui_controller.h>
#include "qnode.h"

namespace Ui {
class Controller;
}

class Controller : public QWidget
{
  Q_OBJECT

public:
  explicit Controller(int argc, char** argv, QWidget *parent = 0);
  ~Controller();

private:
  Ui::Controller *ui;
  QNode *qnode_;

};

#endif // UI_CONTROLLER_H
