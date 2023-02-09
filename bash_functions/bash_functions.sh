ros_workspace=/home/zeid/enpm663_ws

function ros2ws(){
  if [ "$1" = "-b" ]
  then
    source /opt/ros/galactic/setup.zsh
    cd $ros_workspace
    colcon build
    source install/setup.zsh
  elif [ "$1" = "-c" ]
  then
    source /opt/ros/galactic/setup.zsh
    cd $ros_workspace
    rm -rf build install log
    colcon build
    source install/setup.zsh
  else
    source /opt/ros/galactic/setup.zsh
    cd $ros_workspace
    source install/setup.zsh 
    export _colcon_cd_root=/opt/ros/galactic
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
    source /usr/share/colcon_cd/function/colcon_cd.sh
    eval "$(register-python-argcomplete3 ros2)"
    eval "$(register-python-argcomplete3 colcon)"
    export TURTLEBOT3_MODEL=waffle 
  fi
}

function ros1ws(){
  source /opt/ros/noetic/setup.bash
  source /home/zeid/809e_summer2022_final_ws/devel/setup.bash
}

ros2ws