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

#include "vrpn_mocap/tracker.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <string>

namespace vrpn_mocap
{

  using geometry_msgs::msg::AccelStamped;
  using geometry_msgs::msg::PoseStamped;
  using geometry_msgs::msg::TwistStamped;
  using namespace std::chrono_literals;

  std::string Tracker::ValidNodeName(const std::string &tracker_name)
  {
    // replace non alphanum characters with _
    const std::string alnum_name = std::regex_replace(tracker_name, std::regex("[^a-zA-Z0-9_]"), "_");
    // strip consecutive underscores
    const std::string node_name = std::regex_replace(alnum_name, std::regex("_+"), "_");

    return node_name;
  }

  Tracker::Tracker(const std::string &tracker_name)
      : Node(ValidNodeName(tracker_name)),
        name_(tracker_name),
        multi_sensor_(declare_parameter("multi_sensor", false)),
        frame_id_(declare_parameter("frame_id", "world")),
        sensor_data_qos_(declare_parameter("sensor_data_qos", true)),
        pub_idle_timeout_(declare_parameter("pub_idle_timeout", 60.0)),
        vrpn_tracker_(name_.c_str())
  {
    Init();

    // start main loop when instantiated as a standalone node
    const double update_freq = this->declare_parameter("update_freq", 100.);
    timer_ = this->create_wall_timer(1s / update_freq, std::bind(&Tracker::MainLoop, this));
  }

  Tracker::Tracker(
      const rclcpp::Node &base_node, const std::string &tracker_name,
      const std::shared_ptr<vrpn_Connection> &connection)
      : Node(base_node, ValidNodeName(tracker_name)),
        name_(tracker_name),
        multi_sensor_(base_node.get_parameter("multi_sensor").as_bool()),
        frame_id_(base_node.get_parameter("frame_id").as_string()),
        sensor_data_qos_(base_node.get_parameter("sensor_data_qos").as_bool()),
        pub_idle_timeout_(base_node.get_parameter("pub_idle_timeout").as_double()),
        vrpn_tracker_(name_.c_str(), connection.get())
  {
    Init();
  }

  Tracker::~Tracker()
  {
    vrpn_tracker_.unregister_change_handler(this, &Tracker::HandlePose);
    vrpn_tracker_.unregister_change_handler(this, &Tracker::HandleTwist);
    vrpn_tracker_.unregister_change_handler(this, &Tracker::HandleAccel);

    // 停止 publisher 创建线程
    running_ = false;
    pubs_cv_.notify_all();
    if (publisher_thread_.joinable())
    {
      publisher_thread_.join();
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Destroyed new tracker " << name_);
  }

  void Tracker::Init()
  {
    vrpn_tracker_.register_change_handler(this, &Tracker::HandlePose);
    vrpn_tracker_.register_change_handler(this, &Tracker::HandleTwist);
    vrpn_tracker_.register_change_handler(this, &Tracker::HandleAccel);
    vrpn_tracker_.shutup = true;

    // 启动 publisher 异步创建线程
    publisher_thread_ = std::thread(&Tracker::PublisherCreateLoop, this);

    RCLCPP_INFO_STREAM(this->get_logger(), "Created new tracker " << name_);
  }

  void Tracker::MainLoop() { vrpn_tracker_.mainloop(); }

  void Tracker::PublisherCreateLoop()
  {
    // 清理扫描间隔:取 5s 或 idle_timeout/2 中较小者,保证至少 idle/2 内能检测一次
    const double cleanup_interval_sec =
        pub_idle_timeout_ > 0.0 ? std::min(5.0, pub_idle_timeout_ / 2.0) : 5.0;
    const auto cleanup_interval =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cleanup_interval_sec));

    while (running_.load(std::memory_order_acquire))
    {
      std::function<void()> task;
      bool has_task = false;
      {
        std::unique_lock<std::mutex> lk(pubs_mutex_);
        // 用 wait_for 周期性唤醒,既能及时处理创建任务,也能定期执行空闲清理
        pubs_cv_.wait_for(lk, cleanup_interval, [this]() {
          return !create_tasks_.empty() || !running_.load(std::memory_order_acquire);
        });
        if (!running_.load(std::memory_order_acquire) && create_tasks_.empty())
        {
          break;
        }
        if (!create_tasks_.empty())
        {
          task = std::move(create_tasks_.front());
          create_tasks_.pop();
          has_task = true;
        }
      }
      // 在锁外执行 create_publisher,避免阻塞 mainloop 线程的 publish 路径
      if (has_task)
      {
        task();
      }

      // 清理空闲超过 pub_idle_timeout_ 的 publisher
      // 锁内只把要销毁的 publisher std::move 到局部 vector,真正的析构(触发 DDS fini)
      // 留到锁释放后由局部 vector 析构执行,避免阻塞 mainloop 线程的 publish 路径
      std::vector<PublisherT<PoseStamped>::SharedPtr> pose_to_destroy;
      std::vector<PublisherT<TwistStamped>::SharedPtr> twist_to_destroy;
      std::vector<PublisherT<AccelStamped>::SharedPtr> accel_to_destroy;
      {
        std::lock_guard<std::mutex> lk(pubs_mutex_);
        CleanupIdlePublishers<PoseStamped>(&pose_pubs_, &pose_last_recv_, "pose", &pose_to_destroy);
        CleanupIdlePublishers<TwistStamped>(&twist_pubs_, &twist_last_recv_, "velocity", &twist_to_destroy);
        CleanupIdlePublishers<AccelStamped>(&accel_pubs_, &accel_last_recv_, "accel", &accel_to_destroy);
      }
      // 锁外析构:DDS DataWriter 销毁发生在 publisher_thread_,不影响 mainloop 数据接收
      pose_to_destroy.clear();
      twist_to_destroy.clear();
      accel_to_destroy.clear();
    }
  }

  void VRPN_CALLBACK Tracker::HandlePose(void *data, const vrpn_TRACKERCB tracker_pose)
  {
    Tracker *tracker = static_cast<Tracker *>(data);

    // 帧间隔统计：每秒打印一次该秒内的最大/最小间隔（ms）
	static size_t frame_count = 0;
	static rclcpp::Time prev_recv_time;
	static rclcpp::Time sec_start_time;          // 当前秒的开始时间
	static double sec_min = 0.0, sec_max = 0.0; // 当前秒内的极值
	static bool has_interval = false;           // 当前秒是否已有间隔记录
	static size_t sec_frame_count = 0;          // 当前秒内参与统计的帧数（间隔数）

	const rclcpp::Time recv_time = tracker->get_clock()->now();
	double cur_interval_ms = 0.0;

	if (frame_count > 0)
	{
		cur_interval_ms = (recv_time - prev_recv_time).seconds() * 1000.0;

		// 更新当前秒的极值
		if (!has_interval)
		{
			sec_min = sec_max = cur_interval_ms;
			has_interval = true;
			sec_frame_count = 1;
		}
		else
		{
			if (cur_interval_ms < sec_min) sec_min = cur_interval_ms;
			if (cur_interval_ms > sec_max) sec_max = cur_interval_ms;
			sec_frame_count++;
		}

		// 检查是否已过 1 秒，若是则打印并重置
		if ((recv_time - sec_start_time).seconds() >= 1.0)
		{
			RCLCPP_INFO(
			tracker->get_logger(),
				"[Pose] frame=%zu sensor=%d recv=%d.%09d sec_min=%.3fms sec_max=%.3fms sec_frames=%zu",
				frame_count,
				tracker_pose.sensor,
				static_cast<int>(recv_time.seconds()),
				static_cast<int>(recv_time.nanoseconds() % 1000000000LL),
				sec_min,
				sec_max,
				sec_frame_count);

			// 重置新秒，并将当前帧间隔作为新秒的第一个值
			sec_start_time = recv_time;
			sec_min = sec_max = cur_interval_ms;
			has_interval = true;
			sec_frame_count = 1;
		}
	}
	else
	{
		// 首帧：无间隔，仅初始化秒开始时间
		sec_start_time = recv_time;
		has_interval = false;
		sec_frame_count = 0;
	}

	prev_recv_time = recv_time;
	frame_count++;

    // 异步获取 publisher:若尚未就绪(publisher_thread_ 正在创建中),返回 nullptr,丢弃这一帧
    auto pub = tracker->GetOrCreatePublisher<PoseStamped>(
        static_cast<size_t>(tracker_pose.sensor), "pose",
        &tracker->pose_pubs_, &tracker->pose_last_recv_);
    if (!pub)
    {
      return;
    }

    // populate message
    PoseStamped msg;
    msg.header.frame_id = tracker->frame_id_;
    msg.header.stamp = tracker->get_clock()->now();

    msg.pose.position.x = tracker_pose.pos[0];
    msg.pose.position.y = tracker_pose.pos[1];
    msg.pose.position.z = tracker_pose.pos[2];

    msg.pose.orientation.x = tracker_pose.quat[0];
    msg.pose.orientation.y = tracker_pose.quat[1];
    msg.pose.orientation.z = tracker_pose.quat[2];
    msg.pose.orientation.w = tracker_pose.quat[3];

    pub->publish(msg);
  }

  void VRPN_CALLBACK Tracker::HandleTwist(void *data, const vrpn_TRACKERVELCB tracker_twist)
  {
    Tracker *tracker = static_cast<Tracker *>(data);

    // 异步获取 publisher:若尚未就绪,丢弃这一帧
    auto pub = tracker->GetOrCreatePublisher<TwistStamped>(
        static_cast<size_t>(tracker_twist.sensor), "velocity",
        &tracker->twist_pubs_, &tracker->twist_last_recv_);
    if (!pub)
    {
      return;
    }

    // populate message
    TwistStamped msg;
    msg.header.frame_id = tracker->frame_id_;
    msg.header.stamp = tracker->get_clock()->now();

    msg.twist.linear.x = tracker_twist.vel[0];
    msg.twist.linear.y = tracker_twist.vel[1];
    msg.twist.linear.z = tracker_twist.vel[2];

    const Eigen::Quaterniond quat(
        tracker_twist.vel_quat[3], tracker_twist.vel_quat[0], tracker_twist.vel_quat[1],
        tracker_twist.vel_quat[2]);
    const Eigen::AngleAxisd axis_ang(quat);
    const Eigen::Vector3d rot_vel = axis_ang.axis() * axis_ang.angle() / tracker_twist.vel_quat_dt;
    msg.twist.angular.x = rot_vel.x();
    msg.twist.angular.y = rot_vel.y();
    msg.twist.angular.z = rot_vel.z();

    pub->publish(msg);
  }

  void VRPN_CALLBACK Tracker::HandleAccel(void *data, const vrpn_TRACKERACCCB tracker_accel)
  {
    Tracker *tracker = static_cast<Tracker *>(data);

    // 异步获取 publisher:若尚未就绪,丢弃这一帧
    auto pub = tracker->GetOrCreatePublisher<AccelStamped>(
        static_cast<size_t>(tracker_accel.sensor), "accel",
        &tracker->accel_pubs_, &tracker->accel_last_recv_);
    if (!pub)
    {
      return;
    }

    // populate message
    AccelStamped msg;
    msg.header.frame_id = tracker->frame_id_;
    msg.header.stamp = tracker->get_clock()->now();

    msg.accel.linear.x = tracker_accel.acc[0];
    msg.accel.linear.y = tracker_accel.acc[1];
    msg.accel.linear.z = tracker_accel.acc[2];

    const Eigen::Quaterniond quat(
        tracker_accel.acc_quat[3], tracker_accel.acc_quat[0], tracker_accel.acc_quat[1],
        tracker_accel.acc_quat[2]);
    const Eigen::AngleAxisd axis_ang(quat);
    const Eigen::Vector3d rot_acc = axis_ang.axis() * axis_ang.angle() / tracker_accel.acc_quat_dt;
    msg.accel.angular.x = rot_acc.x();
    msg.accel.angular.y = rot_acc.y();
    msg.accel.angular.z = rot_acc.z();

    pub->publish(msg);
  }

} // namespace vrpn_mocap
