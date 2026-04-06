#include <iostream>
#include <memory>

#include "idl_msgs/srv/query.hpp"

int main() {
  auto request = std::make_shared<idl_msgs::srv::Query::Request>();
  request->query = "query";
  std::cout << "Successfully instantiated Query with query: " << request->query
            << std::endl;
  return 0;
}
