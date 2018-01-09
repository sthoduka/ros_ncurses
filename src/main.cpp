#include <ros_ncurses/ncurses_node.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_ncurses");

    NcursesNode nn;
    nn.run();
    return 0;
}
