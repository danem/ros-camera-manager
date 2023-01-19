#include <ros_camera_manager/client.h>


namespace ros_camera_manager {

using namespace ros_camera_manager_msgs::srv;
using namespace ros_camera_manager_msgs::msg;

std::shared_future<Client::QueryResponsePtr> Client::queryAsync (const QueryCamera::Request::SharedPtr& request) {
    return _queryCamera.async_send_request(request);
}

std::shared_future<Client::QueryResponsePtr> Client::queryAsync (
    const std::string& query,
    uint8_t query_type
) {
    auto req = std::make_shared<QueryCamera::Request>();
    req->query_type = query_type;
    req->query = query;
    return _queryCamera.async_send_request(req);
}

Client::QueryResponsePtr Client::query (const QueryCamera::Request::SharedPtr& request) {
    auto resp = queryAsync(request);
    rclcpp::spin_until_future_complete(_node, resp);
    return resp.get();
}

Client::QueryResponsePtr Client::query (const std::string& query, uint8_t query_type) {
    auto resp = queryAsync(query, query_type);
    rclcpp::spin_until_future_complete(_node, resp);
    return resp.get();
}

std::shared_future<Client::StatusResponsePtr> Client::statusAsync () {
    auto req = std::make_shared<GetStatus::Request>();
    return _getStatus.async_send_request(req);
}

Client::StatusResponsePtr Client::status () {
    auto resp = statusAsync();
    rclcpp::spin_until_future_complete(_node, resp);
    return resp.get();
}

Client::RegisterResponsePtr Client::registerCamera (const CameraInfo& camera) {
    auto req = std::make_shared<RegisterCamera::Request>();
    req->camera = camera;
    auto resp = _registerCamera.async_send_request(req);
    rclcpp::spin_until_future_complete(_node, resp);
    return resp.get();
}

QueryCamera::Request::SharedPtr nameQuery (const std::string& name) {
    auto req = std::make_shared<QueryCamera::Request>();
    req->query = name;
    req->query_type = QueryCamera::Request::NAME_QUERY;
    return req;
}

QueryCamera::Request::SharedPtr driverQuery (const std::string& driver) {
    auto req = std::make_shared<QueryCamera::Request>();
    req->query = driver;
    req->query_type = QueryCamera::Request::DRIVER_QUERY;
    return req;
}

QueryCamera::Request::SharedPtr idQuery (const std::string& id) {
    auto req = std::make_shared<QueryCamera::Request>();
    req->query = id;
    req->query_type = QueryCamera::Request::ID_QUERY;
    return req;
}


}