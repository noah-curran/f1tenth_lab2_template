cd /home/ntcurran/Code/f1tenth_labs/f1tenth_lab2_template/f1fenth_gym
pip3 install -e .

cd /home/ntcurran/Code/f1tenth_labs/f1tenth_lab2_template/f1tenth_gym_ros
sudo mkdir -p sim_ws/src/f1tenth_gym_ros
sudo cp -r config/ sim_ws/src/f1tenth_gym_ros/config 
sudo cp -r f1tenth_gym_ros/ sim_ws/src/f1tenth_gym_ros/f1tenth_gym_ros 
sudo cp -r launch/ sim_ws/src/f1tenth_gym_ros/launch 
sudo cp -r maps/ sim_ws/src/f1tenth_gym_ros/maps 
sudo cp -r resource/ sim_ws/src/f1tenth_gym_ros/resource 
sudo cp -r test/ sim_ws/src/f1tenth_gym_ros/test 
sudo cp -r package.xml sim_ws/src/f1tenth_gym_ros/package.xml 
sudo cp -r setup.cfg sim_ws/src/f1tenth_gym_ros/setup.cfg 
sudo cp -r setup.py sim_ws/src/f1tenth_gym_ros/setup.py 

source /opt/ros/foxy/setup.bash
cd /home/ntcurran/Code/f1tenth_labs/f1tenth_lab2_template/f1tenth_gym_ros/sim_ws/
sudo apt-get update --fix-missing
sudo rosdep install -i --from-path src --rosdistro foxy -y
sudo colcon build --paths /home/ntcurran/Code/f1tenth_labs/f1tenth_lab2_template/f1tenth_gym_ros/sim_ws/