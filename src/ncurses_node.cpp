#include <ros_ncurses/ncurses_node.h>


#include <ros/master.h>
#include <ros/ros.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string.h>


NcursesNode::NcursesNode() :
    continuous_refresh_(false),
    ros_master_uri_(std::getenv("ROS_MASTER_URI")),
    ros_ip_(std::getenv("ROS_IP")),
    ros_hostname_(std::getenv("ROS_HOSTNAME"))
{
    initNcurses();
}

NcursesNode::~NcursesNode()
{
    delwin(command_window_);
    delwin(main_window_);
    delwin(status_window_);
    endwin();
}

void NcursesNode::initNcurses()
{
    initscr();
    cbreak();
    raw();
    keypad(stdscr, TRUE);
    noecho();
    curs_set(0);
    timeout(0);
    refresh();
    getmaxyx(stdscr, screen_rows_, screen_cols_);

    command_window_ = newwin(1, screen_cols_, 0, 0);
    wrefresh(command_window_);
    wattron(command_window_, A_DIM);

    main_window_ = newwin(screen_rows_ - 2, screen_cols_, 1, 0);
    wrefresh(main_window_);

    status_window_ = newwin(1, screen_cols_, screen_rows_ - 1, 0);
    wrefresh(status_window_);

    std::string ros_status = ros::master::check()?std::string("True "):std::string("False");
    std::string ros_master_uri_str = "";
    std::string ros_ip_str = "";
    std::string ros_hostname_str = "";
    if (ros_master_uri_)
        ros_master_uri_str = std::string(ros_master_uri_);
    if (ros_ip_)
        ros_ip_str = std::string(ros_ip_);
    if (ros_hostname_)
        ros_hostname_str = std::string(ros_hostname_);
    std::string status = "ROS: " + ros_status + ", ROS_MASTER_URI: " + ros_master_uri_str +
                         ", ROS_IP: " + ros_ip_str + ", ROS_HOSTNAME: " + ros_hostname_str;
    writeStatus(status);

    refresh();
}

void NcursesNode::writeCommand(const std::string &command)
{
    werase(command_window_);
    mvwprintw(command_window_, 0, 0, command.c_str());
    wrefresh(command_window_);
}

void NcursesNode::writeStatus(const std::string &status)
{
    mvwprintw(status_window_, 0, 0, status.c_str());
    wrefresh(status_window_);
}

void NcursesNode::writeOutput()
{
    werase(main_window_);
    wmove(main_window_, 0, 0);
    while (fgets(buffer_, BUFFER_SIZE, command_pipe_.get()) != nullptr)
    {
        wprintw(main_window_, buffer_);
        wrefresh(main_window_);
    }
}

void NcursesNode::updateOutput()
{
    if (fgets(buffer_, BUFFER_SIZE, command_pipe_.get()) != nullptr)
    {
        wprintw(main_window_, buffer_);
        wrefresh(main_window_);
    }
}

void NcursesNode::runCommand(const std::string &command)
{
    command_pipe_ = std::shared_ptr<FILE>(popen(command.c_str(), "r"), pclose);
    if (!command_pipe_)
    {
        throw std::runtime_error("popen() failed.");
    }
}

void NcursesNode::highlightNextRow()
{
    if (current_row_ >= 0)
    {
        mvwchgat(main_window_, current_row_, 0, -1, A_NORMAL, 1, NULL);
    }
    mvwchgat(main_window_, current_row_ + 1, 0, -1, A_REVERSE, 1, NULL);
    current_row_++;
    wrefresh(main_window_);
    continuous_refresh_ = false;
}

void NcursesNode::highlightPreviousRow()
{
    mvwchgat(main_window_, current_row_, 0, -1, A_NORMAL, 1, NULL);
    mvwchgat(main_window_, current_row_ - 1, 0, -1, A_REVERSE, 1, NULL);
    if (current_row_ >= 0)
    {
        current_row_--;
    }
    wrefresh(main_window_);
    continuous_refresh_ = false;
}

void NcursesNode::parseInput(char input, std::string &command)
{
    if (input == ERR)
    {
        // if continuous refresh is true, keep the previous command
        if (!continuous_refresh_)
        {
            command = "";
        }
        return;
    }
    if(input == 'n')
    {
        command = "rosnode list";
        continuous_refresh_ = true;
        mode_ = NODE_LIST;
        current_row_ = -1;
    }
    else if (input == 't')
    {
        command = "rostopic list";
        continuous_refresh_ = true;
        mode_ = TOPIC_LIST;
        current_row_ = -1;
    }
    else if (input == 's')
    {
        command = "rosservice list";
        continuous_refresh_ = true;
        mode_ = SERVICE_LIST;
        current_row_ = -1;
    }
    else if (input == 'j')
    {
        command = "";
        highlightNextRow();
    }
    else if (input == 'k')
    {
        command = "";
        highlightPreviousRow();
    }
    else if (input == 'i')
    {
        int row = current_row_ >= 0 ? current_row_ : 0;
        char line_buffer[128];
        mvwinstr(main_window_, row, 0, line_buffer);
        if (strlen(line_buffer) == 0)
        {
            return;
        }
        else if (mode_ == TOPIC_LIST)
        {
            command = "rostopic info " + std::string(line_buffer);
            mode_ = TOPIC_INFO;
        }
        else if (mode_ == NODE_LIST)
        {
            command = "rosnode info " + std::string(line_buffer);
            mode_ = NODE_INFO;
        }
        else if (mode_ == SERVICE_LIST)
        {
            command = "rosservice info " + std::string(line_buffer);
            mode_ = SERVICE_INFO;
        }
        else
        {
            return;
        }
        current_row_ = -1;
        continuous_refresh_ = true;
    }
}

void NcursesNode::run()
{
    std::string command = "";
    while (true)
    {
        char input = getch();
        if (input == 'q')
        {
            break;
        }

        bool is_ros_alive = ros::master::check();

        std::string ros_status = is_ros_alive?std::string("True "):std::string("False");
        writeStatus("ROS: " + ros_status);
        if (!is_ros_alive)
        {
            continue;
        }
        parseInput(input, command);

        if (!command.empty())
        {
            runCommand(command);
            writeCommand(command);
        }
        if (command_pipe_ && !feof(command_pipe_.get()))
        {
            if (continuous_refresh_)
            {
                writeOutput();
            }
            else
            {
                updateOutput();
            }
        }

    }
}
