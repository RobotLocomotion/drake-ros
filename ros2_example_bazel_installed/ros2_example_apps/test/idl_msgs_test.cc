#include <iostream>
#include <memory>

#include "idl_msgs/srv/query.hpp"

int main() {
  auto query_request = std::make_shared<idl_msgs::srv::Query::Request>();
  query_request->query = "test";
  std::cout << "Successfully instantiated Query Request with: "
            << query_request->query << std::endl;
  return 0;
}
