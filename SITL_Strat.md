
gz sim -v4 -r iris_runway.sdf

sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --console

gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"


conda activate yolo

python3 ~/uav_project/src/yolo_app/main.py

python3 ~/uav_project/src/control/main.py 

cd ~/uav_project/src/
python3 -m app.main 