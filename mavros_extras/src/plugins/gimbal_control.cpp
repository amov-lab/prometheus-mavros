/**
 * @brief GIMBAL Control plugin
 * @file gimbal_control.cpp
 * @author Wang ming <betterming.wang@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Wang ming
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/MountControl.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/MountConfigure.h>

namespace mavros
{
	namespace extra_plugins
	{
		//! Mavlink enumerations
		using mavlink::common::MAV_CMD;
		using mavlink::common::MAV_MOUNT_MODE;
		using utils::enum_value;

		/**
 * @brief Mount Control plugin
 *
 * Publishes Mission commands to control the camera or antenna mount.
 * @see command_cb()
 */
		class GimbalControlPlugin : public plugin::PluginBase
		{
		public:
			GimbalControlPlugin() : PluginBase(),
								   nh("~"),
								   mount_nh("~gimbal_control")
			{
			}

			void initialize(UAS &uas_) override
			{
				PluginBase::initialize(uas_);

				command_sub = mount_nh.subscribe("command", 10, &GimbalControlPlugin::command_cb, this);
				mount_orientation_pub = mount_nh.advertise<geometry_msgs::Quaternion>("orientation", 10);
				mount_status_pub = mount_nh.advertise<geometry_msgs::Vector3Stamped>("status", 10);

				// 10秒后启动，获取吊舱控制权
				gimbal_control_power = nh.createWallTimer(ros::WallDuration(10.0),
														  &GimbalControlPlugin::register_gimbal, this);
				// gimbal_control_power.stop();
			}

			// 在消息路由表中，添加信息，使得返回数据能够进行正确的路由返回数据
			Subscriptions get_subscriptions() override
			{
				return {
					make_handler(&GimbalControlPlugin::handle_mount_orientation),
					make_handler(&GimbalControlPlugin::handle_mount_status)};
			}

		private:
			ros::NodeHandle nh;
			ros::NodeHandle mount_nh;
			ros::Subscriber command_sub;
			ros::Publisher mount_orientation_pub;
			ros::Publisher mount_status_pub;

			ros::WallTimer gimbal_control_power;

			/**
	 * @brief Publish the mount orientation
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION
	 * @param msg   the mavlink message
	 * @param mo	received MountOrientation msg
	 */
			void handle_mount_orientation(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GIMBAL_DEVICE_ATTITUDE_STATUS &mo)
			{
				// auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(mo.roll, mo.pitch, mo.yaw) * M_PI / 180.0);
				geometry_msgs::Quaternion quaternion_msg;
				quaternion_msg.w = mo.q[0];
				quaternion_msg.x = mo.q[1];
				quaternion_msg.y = mo.q[2];
				quaternion_msg.z = mo.q[3];
				// tf::quaternionEigenToMsg(mo.q, quaternion_msg);
				mount_orientation_pub.publish(quaternion_msg);
			}

			/**
	 * @brief Publish the mount status
	 *
	 * @param msg   the mavlink message
	 * @param ms	received MountStatus msg
	 */
			void handle_mount_status(const mavlink::mavlink_message_t *, mavlink::ardupilotmega::msg::MOUNT_STATUS &ms)
			{
				geometry_msgs::Vector3Stamped publish_msg;

				publish_msg.header.stamp = ros::Time::now();

				publish_msg.header.frame_id = std::to_string(ms.target_component);

				auto vec = Eigen::Vector3d(ms.pointing_b, ms.pointing_a, ms.pointing_c) * M_PI / 18000.0;
				tf::vectorEigenToMsg(vec, publish_msg.vector);

				mount_status_pub.publish(publish_msg);

				// pointing_X is cdeg
				auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(ms.pointing_b, ms.pointing_a, ms.pointing_c) * M_PI / 18000.0);
				geometry_msgs::Quaternion quaternion_msg;
				tf::quaternionEigenToMsg(q, quaternion_msg);
				mount_orientation_pub.publish(quaternion_msg);
			}

			/**
	 * @brief Send mount control commands to vehicle
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL
	 * @param req	received MountControl msg
	 */
			void command_cb(const mavros_msgs::MountControl::ConstPtr &req)
			{
				mavlink::common::msg::GIMBAL_MANAGER_SET_ATTITUDE cmd{};
				// mavlink::common::msg::GIMBAL_DEVICE_SET_ATTITUDE cmd{};

				cmd.target_system = m_uas->get_tgt_system();
				cmd.target_component = m_uas->get_tgt_component();
				cmd.flags = 0;
				cmd.gimbal_device_id = 0;
				float roll = req->roll, pitch = req->pitch, yaw = req->yaw;
				roll *= M_PI / 180.0f;
				pitch *= M_PI / 180.f;
				yaw *= M_PI / 180.f;
				yaw = yaw > M_PI ? yaw - 2.f * M_PI : yaw;

				// to quaternion
				double cy = cos(yaw * 0.5);
				double sy = sin(yaw * 0.5);
				double cp = cos(pitch * 0.5);
				double sp = sin(pitch * 0.5);
				double cr = cos(roll * 0.5);
				double sr = sin(roll * 0.5);

				cmd.q = {1, 0, 0, 0};
				cmd.q[0] = cr * cp * cy + sr * sp * sy;
				cmd.q[1] = sr * cp * cy - cr * sp * sy;
				cmd.q[2] = cr * sp * cy + sr * cp * sy;
				cmd.q[3] = cr * cp * sy - sr * sp * cy;
				cmd.angular_velocity_x = req->altitude;	 //rad
				cmd.angular_velocity_y = req->latitude;	 //rad
				cmd.angular_velocity_z = req->longitude; //rad

				UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
			}

			void register_gimbal(const ros::WallTimerEvent &event)
			{
				using mavlink::common::MAV_CMD;
				bool ret = false;
				try
				{
					auto client = nh.serviceClient<mavros_msgs::CommandLong>("cmd/command");

					mavros_msgs::CommandLong cmd{};

					// NOTE 不能用广播
					cmd.request.broadcast = false;
					cmd.request.command = enum_value(MAV_CMD::DO_GIMBAL_MANAGER_CONFIGURE);
					cmd.request.confirmation = false;
					cmd.request.param1 = 1;	  // 系统
					cmd.request.param2 = 240; // UDP component,组件

					ret = client.call(cmd);
				}
				catch (ros::InvalidNameException &ex)
				{
					ROS_ERROR_NAMED("mount", "MountConfigure: %s", ex.what());
				}

				ROS_ERROR_COND_NAMED(!ret, "mount", "MountConfigure: command plugin service call failed!");
				if (ret)
				{
					gimbal_control_power.stop();
				}
			}
		};
	} // namespace extra_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GimbalControlPlugin, mavros::plugin::PluginBase)
