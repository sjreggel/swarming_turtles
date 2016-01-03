[ -z "$PS1" ] && return

# Basic options
# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoreboth
OS_NAME_STR=$(uname)

# Macthigs
if [[ "$OS_NAME_STR" == 'Darwin' ]]; then
    nl ~/.bash_eternal_history | sort -rk 2 | uniq -f 1 | sort -n | cut -f 2 > ~/.temp_file
else
    nl ~/.bash_eternal_history | sort -k 2 | uniq -f 1 | sort -n | cut -f 2 > ~/.temp_file
fi

cat ~/.temp_file > ~/.bash_eternal_history
#rm temp_file

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
# Eternal bash history.
# ---------------------
# Undocumented feature which sets the size to "unlimited".
# http://stackoverflow.com/questions/9457233/unlimited-bash-history
export HISTFILESIZE=
export HISTSIZE=
#export HISTTIMEFORMAT="[%F %T] "
# Change the file location because certain bash sessions truncate .bash_history file upon close.
# http://superuser.com/questions/575479/bash-history-truncated-to-500-lines-on-each-login
export HISTFILE=~/.bash_eternal_history
# Force prompt to write history after every command.
# http://superuser.com/questions/20900/bash-history-loss
PROMPT_COMMAND="history -a; $PROMPT_COMMAND"


#export HISTCONTROL=ignoredups
export COLORFGBG='default;default'

declare -x CLICOLOR=1
declare -x LSCOLORS="DxGxFxdxCxdxdxhbadExEx"

# Aliases
alias ..='cd ..'
alias cd..='cd ..'
alias ...='cd ../..'
alias back='cd $OLDPWD'
alias dfh='df -h'
alias 'll'='ls -la'

# Use custom aliases
if [ -f ~/.bash_aliases ]; then
. ~/.bash_aliases
fi

# Prompt
BGREEN='\[\033[1;32m\]'
GREEN='\[\033[0;32m\]'
BRED='\[\033[1;31m\]'
RED='\[\033[0;31m\]'
BBLUE='\[\033[1;34m\]'
BLUE='\[\033[0;34m\]'
NORMAL='\[\033[00m\]'
BYELLOW='\[\033[1;33m\]'
PS1="${BRED}\u${NORMAL}@${BBLUE}\h${NORMAL}:${BGREEN}\w${NORMAL} "

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    eval "`dircolors -b`"
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

if [ -f /etc/bash_completion ]; then
    source /etc/bash_completion
fi


# make bash autocomplete with up arrow
bind '"\e[A":history-search-backward'
bind '"\e[B":history-search-forward'


#------------------------------------------------------------------------------
# ROS
#------------------------------------------------------------------------------

if [ -x /opt/ros/indigo ]; then
    export TURTLEBOT_3D_SENSOR=kinect
    export TURTLEBOT_BATTERY=/sys/class/power_supply/BAT1
    source /opt/ros/indigo/setup.bash
    source ~/ros/devel/setup.bash

elif [ -x ~/ros_hydro_ws/devel_isolated ]; then
    source ~/ros_hydro_ws/install_isolated/setup.bash
    source ~/ros/devel/setup.bash
    
elif [ -x /opt/ros/hydro ]; then
    source /opt/ros/hydro/setup.bash
    source ~/ros/devel/setup.bash
   
fi

export ROS_WORKSPACE=~/ros/src/
export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH

#export EDITOR='gedit'
export EDITOR='emacs'

alias 'rviz'='rosrun rviz rviz'
alias 'dynreconf'='rosrun rqt_reconfigure rqt_reconfigure'
alias 'emacs'='emacs -nw'
alias sudo='sudo '

alias catkin_ws_make='catkin_make -C "$ROS_WORKSPACE/.."'

function copy-ssh-to {
    cat ~/.ssh/id_rsa.pub | ssh $1 'cat >> .ssh/authorized_keys && echo "Key copied"'
}

function ros {
    export ROS_MASTER_URI=http://$1:11311
}

function rospath {
    export ROS_PACKAGE_PATH=$1:$ROS_PACKAGE_PATH
}

function roswspath {
    source /opt/ros/indigo/setup.bash
    source $(pwd $1)/devel/setup.bash
    export ROS_WORKSPACE=$(pwd $1)/src
    export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH
}


# Macthigs
if [[ "$OS_NAME_STR" == 'Darwin' ]]; then
    if [ -f $(brew --prefix)/etc/bash_completion ]; then
	source $(brew --prefix)/etc/bash_completion
    fi
    
    #export LD_LIBRARY_PATH=/usr/local/lib:/usr/local/include/boost:${LD_LIBRARY_PATH}
    export CPATH=/usr/local/include
    export LIBRARY_PATH=/usr/local/lib
    #export DYLD_FALLBACK_LIBRARY_PATH=$DYLD_FALLBACK_LIBRARY_PATH:/usr/local/lib
    export BOOST_HOME=/usr/local/include/boost
    
    export PATH=/usr/local/bin:$PATH

    if [ -x /usr/local/mysql/bin ]; then
	export PATH=/usr/local/mysql/bin:$PATH
    fi
    
    if [ -x /Applications/MATLAB_R2011b.app ]; then
	export PATH=/Applications/MATLAB_R2011b.app/bin:$PATH
    fi
    
    if [ -x /usr/texbin ]; then
	export PATH=/usr/texbin:$PATH
    fi
    export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH
    # OPAM configuration
    . /Volumes/MacData/User/danielclaes/.opam/opam-init/init.sh > /dev/null 2> /dev/null || true
    export PAPARAZZI_HOME=~/paparazzi
    export PAPARAZZI_SRC=~/paparazzi
    export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/opt/X11/lib/pkgconfig
    
    
else
    #ros slaw
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/opt/softkinetic/DepthSenseSDK/lib    
fi

# Enable tab completion
source ~/git-completion.bash


# Change command prompt
source ~/git-prompt.sh
export GIT_PS1_SHOWDIRTYSTATE=1
# '\u' adds the name of the current user to the prompt
# '\$(__git_ps1)' adds git-related stuff
# '\W' adds the name of the current directory
#export PS1="$purple\u$green\$(__git_ps1)$blue \W $ $reset"
PS1="${BRED}\u${NORMAL}@${BBLUE}\h$BYELLOW\$(__git_ps1)${NORMAL}:${BGREEN}\w${NORMAL} "

