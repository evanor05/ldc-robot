#include "ldc_qt_gui/main_window.hpp"
#include "ldc_qt_gui/ros_interface.hpp"

#include <QApplication>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  setenv("QT_QPA_PLATFORM", "xcb", 0);
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  QTimer shutdown_timer;
  QObject::connect(&shutdown_timer, &QTimer::timeout, &app, [&app]() {
    if (!rclcpp::ok()) app.quit();
  });
  shutdown_timer.start(100);

  auto ros = std::make_shared<ldc_qt_gui::RosInterface>();
  ros->start();

  ldc_qt_gui::MainWindow window(ros);
  window.show();

  const int result = app.exec();
  ros->stop();
  rclcpp::shutdown();
  return result;
}
