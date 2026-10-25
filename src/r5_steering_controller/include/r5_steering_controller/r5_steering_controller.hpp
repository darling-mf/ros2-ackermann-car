#ifndef R5_STEERING_CONTROLLER__R5_STEERING_CONTROLLER_HPP_
#define R5_STEERING_CONTROLLER__R5_STEERING_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "ackermann_steering_controller/ackermann_steering_controller.hpp"

namespace r5_steering_controller
{

/**
 * \brief TARKBOT R5 阿克曼控制器。
 *
 * 继承 ackermann_steering_controller（复用全部运动学与接口管理），
 * 在“父类写完命令之后、硬件读取之前”叠加三个实车刚需的约束，
 * 全部实现为【命令层后处理】：
 *
 *  1. 驱动轮速度斜坡（加速度限幅）：AT8236 板载限流 1.3A，阶跃速度
 *     指令会造成电流尖峰；直行时左右轮同速，斜坡整形后底盘加速度
 *     精确等于 max_accel。
 *  2. 速度相关转向限幅：满舵时 a_y = v²/R。v=1.2 m/s、R=0.55 时
 *     a_y ≈ 2.6 m/s²（0.27g），小轮胎接近滑移边界；限 a_lat_max 后
 *     高速自动收舵、低速不受影响。
 *  3. 转向速率限幅：舵机 S620F-180 响应 0.11s/60° ≈ 9.5 rad/s，
 *     且 Gazebo 的 position 接口“瞬移”跟踪，不限速率时仿真轨迹
 *     会比实车激进。
 *
 * ⚠️ 为什么放在命令层而不是参考层（这是本包最重要的设计教训）：
 *   非链式模式下，父类 update_and_write_commands 开头无条件从
 *   input_ref_ 重载 reference_interfaces_，结尾又把它们清成 NaN。
 *   子类在调用父类【之前】写进 reference_interfaces_ 的任何限幅值
 *   都会被覆盖（第一版实测峰值加速度 2.48 m/s²，限幅形同虚设，
 *   是验收测试抓出来的）。而命令接口的值在 update 返回后才被
 *   resource_manager 读取送往硬件，update 线程内“读出→限幅→写回”
 *   是安全且确定生效的。
 *
 * 近似说明：转向限幅用自行车模型公式作用在轮角上（真实内轮角比
 * 虚拟中心角大 ~17%），作为保护性限幅足够；几何精确性由父类的
 * max_steering_angle（已换算虚拟中心角）保证。
 */
class R5SteeringController : public ackermann_steering_controller::AckermannSteeringController
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ===== 参数（on_init 里声明并读取）=====
  double max_accel_ = 0.8;         // r5.max_accel       底盘最大加速度 [m/s^2]
  double max_decel_ = 1.5;         // r5.max_decel       底盘最大减速度 [m/s^2]（正值）
  double a_lat_max_ = 2.0;         // r5.a_lat_max       允许的最大侧向加速度 [m/s^2]
  double max_steer_mech_ = 0.235;  // r5.max_steer_mech  单前轮机械限位 [rad]
  double max_steer_rate_ = 9.5;    // r5.max_steer_rate  转向速率上限 [rad/s]
  bool speed_dependent_limit_ = true;  // r5.speed_dependent_limit

  // ===== 状态 =====
  std::vector<double> last_traction_cmds_;  // 驱动轮上一周期指令（斜坡基准）
  std::vector<double> last_steer_cmds_;     // 转向轮上一周期指令（速率基准）
};

}  // namespace r5_steering_controller

#endif  // R5_STEERING_CONTROLLER__R5_STEERING_CONTROLLER_HPP_
