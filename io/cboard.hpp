#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "io/gimbal/gimbal.hpp"
#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有
  // CBoard(const std::string & config_path);
  CBoard(const std::string & config_path ,const std::string& mode_str);
  ~CBoard();
  void send(Command command);
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  

private:
  std::string current_mode_;
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
    uint16_t crc16;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
   SocketCAN can_;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  void callback(const can_frame & frame);
  

  std::string read_yaml(const std::string & config_path);

  
  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;
  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__CBOARD_HPP