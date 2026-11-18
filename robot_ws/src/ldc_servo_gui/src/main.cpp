#include "ldc_servo_gui/main_window.hpp"

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

  auto node = std::make_shared<ldc_servo_gui::ServoPublisher>();
  ldc_servo_gui::MainWindow window(node);
  window.show();

  const int result = app.exec();
  rclcpp::shutdown();
  return result;
}
