#include <px4_trajectory_replanning/ui/controller.h>
#include <QApplication>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  OffboardController controller(argc, argv);
  controller.show();
  return app.exec();
}
