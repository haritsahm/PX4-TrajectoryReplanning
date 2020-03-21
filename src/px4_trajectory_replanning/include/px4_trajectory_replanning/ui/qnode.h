#ifndef QNODE_H
#define QNODE_H

#ifndef Q_MOC_RUN

#include <QThread>
#include <stdio.h>

#include <ros/ros.h>
#include <ros/package.h>

#endif

class QNode : public QThread
{
  Q_OBJECT

public:
  QNode(int argc, char** argv);
  virtual ~QNode();

  bool init();
  void run();

signals:
  void rosShutdown();

private:
  int init_argc_;
  char** init_argv_;
  bool debug_;
};

#endif // QNODE_H
