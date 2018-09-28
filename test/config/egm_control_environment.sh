#!/bin/bash

thisFile=$_
if [ $BASH ]
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_EGM_CONTROL_BASE()
{
  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$thisFile")/.." && pwd)"

  # different cases for software/config or software/build/config
  case "$(basename $configParentDir)" in
    "software") export EGM_CONTROL_BASE=$(dirname $configParentDir);;
    "build") export EGM_CONTROL_BASE=$(dirname $(dirname $configParentDir));;
    *) echo "Warning: egm_control environment file is stored in unrecognized location: $thisFile";;
  esac
  export egm_controlDATA_BASE=$EGM_CONTROL_BASE/../egm_controldata
  export PATH=$PATH:$EGM_CONTROL_BASE/software/build/bin
}

setup_egm_control()
{
  export PATH=$PATH:$EGM_CONTROL_BASE/software/build/bin
  export PATH=$PATH:$HOME/software/libbot/build/bin  # for lcm and libbot install
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$EGM_CONTROL_BASE/software/build/lib:$EGM_CONTROL_BASE/software/build/lib64:$LD_LIBRARY_PATH
  #~ export LD_LIBRARY_PATH=$HOME/software/gurobi702/linux64/lib:$LD_LIBRARY_PATH  # for gurobi

  export GUROBI_HOME="$HOME/software/gurobi702/linux64"
  export PATH="${PATH}:${GUROBI_HOME}/bin"
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"

  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$EGM_CONTROL_BASE/software/build/share/java/lcmtypes_egm_control_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$EGM_CONTROL_BASE/software/build/share/java/drake.jar:$EGM_CONTROL_BASE/software/build/share/java/bot2-lcmgl.jar
  export PKG_CONFIG_PATH=$EGM_CONTROL_BASE/software/build/lib/pkgconfig:$EGM_CONTROL_BASE/software/build/lib64/pkgconfig:$PKG_CONFIG_PATH
  export GRB_LICENSE_FILE=$HOME/gurobi.lic
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cudnn/v6.0/lib64
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-8.0/lib64:/usr/local/cuda-8.0/extras/CUPTI/lib64
  # python path

  # python path
  export PYTHONPATH=$PYTHONPATH:$EGM_CONTROL_BASE/software/build/lib/python2.7/site-packages:$EGM_CONTROL_BASE/software/build/lib/python2.7/dist-packages
  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"

  export PATH=$PATH:$HOME/software/ffmpeg-2.4.2-64bit-static # for ffmpeg software

  export ROSLAUNCH_SSH_UNKNOWN=1
}

set_ros()
{
  if [ -f $EGM_CONTROL_BASE/catkin_ws/devel/setup.bash ]; then
    source $EGM_CONTROL_BASE/catkin_ws/devel/setup.bash
    echo "Setting egm_control environment"
  else
    source /opt/ros/kinetic/setup.bash
  fi
  export ROS_PACKAGE_PATH=$HOME/egm_control/ros_ws/:$ROS_PACKAGE_PATH
}

echo "Memory available:"
df -h --output=avail /
# some useful commands
alias cdegm_control='cd $EGM_CONTROL_BASE'
alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $EGM_CONTROL_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias yolo='rosservice call /robot2_SetSpeed 1600 180'
alias faster='rosservice call /robot2_SetSpeed 200 50'
alias fast='rosservice call /robot2_SetSpeed 100 30'
alias slow='rosservice call /robot2_SetSpeed 50 15'

alias gohome='rosservice call robot2_SetJoints "{j1: 0, j2: 0, j3: 0, j4: 0, j5: 90, j6: 0}"'

alias teleop='rosrun teleop teleop'
alias pythonegm_control='ipython -i -c "run $EGM_CONTROL_BASE/catkin_ws/src/egm_control_config/python/pythonegm_control.py"'

alias pman='bot-procman-sheriff -l $EGM_CONTROL_BASE/software/config/egm_control.pmd'

alias roslocal='export ROS_MASTER_URI=http://localhost:11311'

alias getjoint='rosservice call -- robot2_GetJoints'
alias getcart='rosservice call -- robot2_GetCartesian'
alias setjoint='rosservice call -- robot2_SetJoints'
alias setcart='rosservice call -- robot2_SetCartesian'
alias setspeed='rosservice call /robot2_SetSpeed'
alias zeroft='rosservice call zero'

alias lcmlocal='sudo ifconfig lo multicast; sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo'


alias catmake='cd $EGM_CONTROL_BASE/catkin_ws; catkin_make; cd -;'
alias catrun='cd $EGM_CONTROL_BASE/catkin_ws; catkin_make -DCMAKE_CXX_FLAGS=-Ofast; cd -;'
alias catsim='cd $EGM_CONTROL_BASE/catkin_ws; catkin_make -DCMAKE_CXX_FLAGS=-Ofast -DCATKIN_BLACKLIST_PACKAGES="apriltags;realsense-ros;abb-ros-catkin;irb120_moveit_config;vicon_bridge"; cd -;'
alias simrun='cd $EGM_CONTROL_BASE;  rosrun  push_control push_control'
alias realrun='cd $EGM_CONTROL_BASE/catkin_ws; catkin_make && rosrun  push_control push_control'
alias pythonssh='f(){ FILEPATH_TMP="$1" ; CURRENTPATH="$PWD"; cd $EGM_CONTROL_BASE/../mcube_learning/helper/; FOLDER_NAME=$(python transfer_files.py server "string_as_arg"); cd $EGM_CONTROL_BASE/../mcube_learning/ ;./run_server.sh $CURRENTPATH $FILEPATH_TMP $FOLDER_NAME mcube@192.168.0.191; cd $CURRENTPATH; unset -f f; }; f'
alias pythonssh2='f(){ FILEPATH_TMP="$1" ; CURRENTPATH="$PWD"; cd ../../helper/; FOLDER_NAME=$(python transfer_files.py main "string_as_arg"); cd $CODE_BASE ;./run_server.sh $CURRENTPATH $FILEPATH_TMP $FOLDER_NAME mcube@192.168.0.15; cd $CURRENTPATH; unset -f f; }; f'
alias sshserver='sshpass -p "thecube" ssh mcube@192.168.0.191 -X'
alias sshmain='sshpass -p "thecube" ssh mcube@192.168.0.15 -X'
alias sshfrank='sshpass -p "thecube" ssh mcube@192.168.0.11 -X'
alias sshmaria='sshpass -p "thecube" ssh mcube@192.168.0.223 -X'
alias myip="python -c 'import socket; print([l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith(\"127.\")][:1], [[(s.connect((\"8.8.8.8\", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0])'"

alias s='rosparam set egm_status 0'

ppms2mp4()
{
  bot-ppmsgz $1 mpeg4 10M 30 $1.mp4
}

function lowersuffix {
  cd "$1"
  find . -name '*.*' -exec sh -c '
  a=$(echo {} | sed -r "s/([^.]*)\$/\L\1/");
  [ "$a" != "{}" ] && mv "{}" "$a" ' \;
}

function set_bash {
   PROMPT_COMMAND='history -a'
   history -a

   # sorting in old style
   LC_COLLATE="C"
   export LC_COLLATE

   ulimit -c unlimited
   export HISTTIMEFORMAT="%d/%m/%y %T "
}

set_EGM_CONTROL_BASE
setup_egm_control
set_ros
set_bash

alias ..='cd ..'
alias c='clear'
alias lg='ls | grep'
alias lpy='ls | grep .py'
# some more ls aliases
alias lla='ls -alF'
alias ll='ls -ltrF'
alias llpy='ls -ltrF | grep .py'
alias llg='ls -ltrF | grep'
alias l.='ls -d .* --color=auto'
alias listalias="alias -p | cut -d= -f1 | cut -d' ' -f2"
alias calculator='bc -l'
alias date='date'
alias meminfo='free -m -l -t'
alias gpull='git pull'
alias gpush='git push'
alias gstatus='git status'
alias h='hisotry'
alias hg='history | grep'
alias restartwifi='sudo service network-manager restart'
alias pyfast='python -m pdb' # from Karpathy's twitter
alias volume='pactl -- set-sink-volume 0 '
alias pystart='ipython -i -c "run $CODE_BASE/software/config/pystart.py"'

. /usr/share/autojump/autojump.bash

cd_go()  {
   cd  $(\ls -1dt ./*/ | head -n "$1" | tail -1)
}
