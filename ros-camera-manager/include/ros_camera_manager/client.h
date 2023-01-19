#pragma once

#include <cinttypes>
#include <memory>
#include <string>
#include <future>

#include <ros_camera_manager_msgs/msg/camera_info.hpp>
#include <ros_camera_manager_msgs/msg/camera_stream.hpp>
#include <ros_camera_manager_msgs/srv/get_status.hpp>
#include <ros_camera_manager_msgs/srv/query_camera.hpp>
#include <ros_camera_manager_msgs/srv/register_camera.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros_camera_manager {

using namespace ros_camera_manager_msgs::srv;
using namespace ros_camera_manager_msgs::msg;


class Client {
private:
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Client<QueryCamera> _queryCamera;
    rclcpp::Client<GetStatus> _getStatus;
    rclcpp::Client<RegisterCamera> _registerCamera;

public:
    using QueryResponsePtr = QueryCamera::Response::SharedPtr;
    using StatusResponsePtr = GetStatus::Response::SharedPtr;
    using RegisterResponsePtr =  RegisterCamera::Response::SharedPtr;

    Client ();

    std::shared_future<QueryResponsePtr> queryAsync (const QueryCamera::Request::SharedPtr&);
    std::shared_future<QueryResponsePtr> queryAsync (
        const std::string& query,
        uint8_t query_type
    );

    QueryResponsePtr query (const QueryCamera::Request::SharedPtr&);
    QueryResponsePtr query (const std::string& query, uint8_t query_type);

    std::shared_future<StatusResponsePtr> statusAsync ();
    StatusResponsePtr status ();

    RegisterResponsePtr registerCamera (const CameraInfo& camera);

};

QueryCamera::Request::SharedPtr nameQuery (const std::string& name);
QueryCamera::Request::SharedPtr driverQuery (const std::string& driver);
QueryCamera::Request::SharedPtr idQuery (const std::string& id);




} // end namespace