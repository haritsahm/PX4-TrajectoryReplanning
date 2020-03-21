#include <px4_trajectory_replanning/ui/controller.h>

Controller::Controller(int argc, char** argv, QWidget *parent) :
  QWidget(parent),
  ui(new Ui::Controller)
{
  ui->setupUi(this);

  qnode_->init();

}

Controller::~Controller()
{
  delete ui;
}
