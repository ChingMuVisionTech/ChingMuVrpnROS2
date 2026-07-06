// MIT License
//
// Copyright (c) 2022 Alvin Sun
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef VRPN_MOCAP__TRACKER_HPP_
#define VRPN_MOCAP__TRACKER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "vrpn_Connection.h"
#include "vrpn_Tracker.h"

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vrpn_mocap
{

  /**
   * @brief a ROS2 node for tracking a single object in a VRPN network
   */
  class Tracker : public rclcpp::Node
  {
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(Tracker)

    /**
     * @brief constructor
     *
     * @param tracker_name name of the object to track
     */
    explicit Tracker(const std::string &tracker_name);

    /**
     * @brief destructor
     */
    ~Tracker();

  protected:
    /**
     * @brief single object tracker created only from Client
     *
     * @param base_node VRPNClient node
     * @param name tracker name
     * @param connection vrpn connection pointer (looked up from tracker name if nullptr)
     * @see vrpn_mocap::Client
     */
    Tracker(
        const rclcpp::Node &base_node, const std::string &tracker_name,
        const std::shared_ptr<vrpn_Connection> &connection = nullptr);

  private:
    template <typename MsgT>
    using PublisherT = rclcpp::Publisher<MsgT>;

    template <typename... Args>
    static Tracker::SharedPtr private_make_shared(Args &&...args)
    {
      class TrackerDerived : public Tracker
      {
      public:
        explicit TrackerDerived(Args &&...args)
            : Tracker(std::forward<Args>(args)...) {}
      };

      return std::make_shared<TrackerDerived>(std::forward<Args>(args)...);
    }

    void Init();

    void MainLoop();

    void PublisherCreateLoop();

    std::string ValidNodeName(const std::string &name);

    const std::string name_;
    const bool multi_sensor_;
    const std::string frame_id_;
    const bool sensor_data_qos_;
    const double pub_idle_timeout_; // publisher 空闲超时(秒),超过则清理
    const std::shared_ptr<vrpn_Connection> connection_;

    vrpn_Tracker_Remote vrpn_tracker_;

    std::vector<PublisherT<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
    std::vector<PublisherT<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_pubs_;
    std::vector<PublisherT<geometry_msgs::msg::AccelStamped>::SharedPtr> accel_pubs_;

    // 与 pose_pubs_/twist_pubs_/accel_pubs_ 并行,记录每个 publisher 最后一次 publish 数据的时间
    std::vector<std::chrono::steady_clock::time_point> pose_last_recv_;
    std::vector<std::chrono::steady_clock::time_point> twist_last_recv_;
    std::vector<std::chrono::steady_clock::time_point> accel_last_recv_;

    rclcpp::TimerBase::SharedPtr timer_;

    // 异步创建 publisher 的线程相关成员
    // 目的:避免 create_publisher 阻塞 mainloop 线程,影响其他 tracker 数据接收
    std::mutex pubs_mutex_;
    std::condition_variable pubs_cv_;
    std::queue<std::function<void()>> create_tasks_;
    std::unordered_set<std::string> pending_keys_; // 去重,避免同一 (channel,sensor_idx) 重复派发
    std::atomic<bool> running_{true};
    std::thread publisher_thread_;

    template <typename MsgT>
    typename PublisherT<MsgT>::SharedPtr GetOrCreatePublisher(
        const size_t &sensor_idx, const std::string &channel,
        std::vector<typename PublisherT<MsgT>::SharedPtr> *pubs,
        std::vector<std::chrono::steady_clock::time_point> *last_recv)
    {
      // 快速路径:加锁检查 publisher 是否已就绪,就绪则更新时间戳并返回
      std::lock_guard<std::mutex> lock(pubs_mutex_);
      if (pubs->size() > sensor_idx && pubs->at(sensor_idx))
      {
        if (last_recv->size() <= sensor_idx)
        {
          last_recv->resize(sensor_idx + 1);
        }
        (*last_recv)[sensor_idx] = std::chrono::steady_clock::now();
        return pubs->at(sensor_idx);
      }

      // publisher 未就绪:派发异步创建任务(同一 (channel,sensor_idx) 只派发一次)
      const std::string key = channel + "_" + std::to_string(sensor_idx);
      if (pending_keys_.count(key) == 0)
      {
        pending_keys_.insert(key);
        // 捕获 this/sensor_idx/channel/pubs/last_recv,在独立线程中执行 create_publisher
        create_tasks_.push([this, sensor_idx, channel, pubs, last_recv]() {
          const std::string sensor_channel =
              multi_sensor_ ? channel + "_id_" + std::to_string(sensor_idx) : channel;
          typename PublisherT<MsgT>::SharedPtr pub;
          if (sensor_data_qos_)
          {
            pub = this->create_publisher<MsgT>(sensor_channel, rclcpp::SensorDataQoS());
          }
          else
          {
            pub = this->create_publisher<MsgT>(sensor_channel, rclcpp::SystemDefaultsQoS());
          }
          RCLCPP_INFO_STREAM(this->get_logger(), "Created sensor " << sensor_idx << " channel=" << sensor_channel);

          std::lock_guard<std::mutex> lk(pubs_mutex_);
          if (pubs->size() <= sensor_idx)
          {
            pubs->resize(sensor_idx + 1);
          }
          if (last_recv->size() <= sensor_idx)
          {
            last_recv->resize(sensor_idx + 1);
          }
          pubs->at(sensor_idx) = pub;
          (*last_recv)[sensor_idx] = std::chrono::steady_clock::now();
          pending_keys_.erase(channel + "_" + std::to_string(sensor_idx));
        });
        pubs_cv_.notify_one();
      }

      // publisher 尚未就绪:这一帧丢弃,不阻塞 mainloop 线程
      return nullptr;
    }

    // 清理空闲超过 idle_timeout 的 publisher(在 pubs_mutex_ 已加锁时调用)
    // 锁内只把 publisher std::move 到 to_destroy_out(纳秒级操作),
    // 真正的 rclcpp::Publisher 析构(触发 DDS DataWriter 销毁)留给调用方在锁外执行,
    // 避免 DDS fini 阻塞 mainloop 线程的 publish 路径
    template <typename MsgT>
    void CleanupIdlePublishers(
        std::vector<typename PublisherT<MsgT>::SharedPtr> *pubs,
        std::vector<std::chrono::steady_clock::time_point> *last_recv,
        const std::string &channel,
        std::vector<typename PublisherT<MsgT>::SharedPtr> *to_destroy_out)
    {
      if (pub_idle_timeout_ <= 0.0)
      {
        return; // <=0 表示禁用清理
      }
      const auto now = std::chrono::steady_clock::now();
      const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(pub_idle_timeout_));
      for (size_t i = 0; i < pubs->size(); ++i)
      {
        if (!pubs->at(i))
        {
          continue;
        }
        // last_recv 未初始化时跳过(理论上 publisher 创建时一定写了时间戳)
        if (last_recv->size() <= i)
        {
          continue;
        }
        const auto &t = (*last_recv)[i];
        if (now - t >= timeout)
        {
          const std::string sensor_channel =
              multi_sensor_ ? channel + "_id_" + std::to_string(i) : channel;
          RCLCPP_INFO_STREAM(this->get_logger(), "Cleaning idle publisher sensor=" << i << " channel=" << sensor_channel);
          // 锁内只搬运,析构交给调用方锁外完成
          to_destroy_out->push_back(std::move(pubs->at(i)));
          (*last_recv)[i] = std::chrono::steady_clock::time_point{};
        }
      }
    }

    static void VRPN_CALLBACK HandlePose(void *tracker, const vrpn_TRACKERCB tracker_pose);
    static void VRPN_CALLBACK HandleTwist(void *tracker, const vrpn_TRACKERVELCB tracker_vel);
    static void VRPN_CALLBACK HandleAccel(void *tracker, const vrpn_TRACKERACCCB tracker_acc);

    friend class Client;
  };

} // namespace vrpn_mocap

#endif // VRPN_MOCAP__TRACKER_HPP_
