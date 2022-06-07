mkdir -p models

# Clone UR and checkout melodic-devel-staging
git clone https://github.com/ros-industrial/universal_robot models/universal_robot/
git -C models/universal_robot/ checkout melodic-devel-staging
# Render UR urdfs
bash setup_render_test.sh -d models/universal_robot

# Download and unzip NAO
wget -q https://fuel.gazebosim.org/1.0/OpenRobotics/models/NAO%20with%20Ignition%20position%20controller/1/NAO%20with%20Ignition%20position%20controller.zip --show-progress -P models/
unzip -u models/NAO\ with\ Ignition\ position\ controller.zip -d models/NAO
rm models/NAO\ with\ Ignition\ position\ controller.zip
# Transform and test Shadow
bash setup_render_test.sh -d models/NAO -m model.sdf

# Download and unzip Shadow Hand
wget -q https://fuel.gazebosim.org/1.0/AndrejOrsula/models/shadow_hand/2/shadow_hand.zip --show-progress -P models/
unzip -u models/shadow_hand.zip -d models/shadow_hand
rm models/shadow_hand.zip
# Transform and test Shadow Hand
bash setup_render_test.sh -d models/shadow_hand -m model.sdf

# Download and unzip UR5 rg2
wget -q https://fuel.gazebosim.org/1.0/AndrejOrsula/models/ur5_rg2/2/ur5_rg2.zip --show-progress -P models/
unzip -u models/ur5_rg2.zip -d models/ur5_rg2
rm models/ur5_rg2.zip
# Transform and test UR5
bash setup_render_test.sh -d models/shadow_hand -m model.sdf

# Download and unzip Panda
wget -q https://fuel.gazebosim.org/1.0/OpenRobotics/models/Panda%20with%20Ignition%20position%20controller%20model/2/Panda%20with%20Ignition%20position%20controller%20model.zip --show-progress -P models/
unzip -u models/Panda\ with\ Ignition\ position\ controller\ model.zip -d models/Panda
rm models/Panda\ with\ Ignition\ position\ controller\ model.zip
# Transform and test Panda
bash setup_render_test.sh -d models/Panda -m model.sdf

# Download and unzip Cerberus Anymal
wget -q https://fuel.gazebosim.org/1.0/OpenRobotics/models/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2/6/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2.zip --show-progress -P models/
unzip -u models/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2.zip -d models/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2
rm models/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2.zip
# Transform and test Cerberus Anymal
bash setup_render_test.sh -d models/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2 -m model.sdf

# Download and unzip MPL right arm
wget -q https://fuel.gazebosim.org/1.0/OpenRobotics/models/MPL%20right%20arm/2/MPL%20right%20arm.zip --show-progress -P models/
unzip -u models/MPL\ right\ arm.zip -d models/MPL
rm models/MPL\ right\ arm.zip
# Transform and test MPL righ arm
bash setup_render_test.sh -d models/MPL -m model.sdf
