#include "policy_msgs/srv/policy_reset.hpp"
#include <iostream>

int main(){
    auto request = std::make_shared<policy_msgs::srv::PolicyReset::Request>();
    request->seed = 12;
    std::cout << "Successfully instantiated PolicyReset with seed: " << request->seed << std::endl;
    return 0;
}
