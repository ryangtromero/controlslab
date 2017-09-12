# whirlybird ROS workspace

This is the student workspace for the whirlybird. It contains all the necessary submodules
to run the whirlybird. Before you install the whirlybird workspace, you will first need to 
configure ROS. This is done by running the following commands in a terminal.
``` bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


To install the whirlybird workspace, log into your caedm account, pull up a terminal and 
run the following commands

``` bash
cd ~
git clone https://magiccvs.byu.edu/gitlab/whirlybird/whirlybird_ws.git 
cd whirlybird_ws
git submodule update --init --recursive
catkin_make 
```

Now you will need to add some lines to your `~/.bashrc` in order to connect to the whirlybirds. This is done with
the following commands.

``` bash
echo "source ~/whirlybird_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/whirlybird_ws/whirlybird_lab.bash" >> ~/.bashrc
```
