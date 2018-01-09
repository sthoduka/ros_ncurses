#ifndef NCURSES_NODE_H
#define NCURSES_NODE_H

#include <string>
#include <memory>
#include <ncurses.h>

class NcursesNode
{
private:
    int screen_rows_;
    int screen_cols_;
    bool continuous_refresh_;
    int current_row_;

    const char * ros_master_uri_;
    const char * ros_ip_;
    const char * ros_hostname_;

    const static int BUFFER_SIZE = 128;
    char buffer_[BUFFER_SIZE];

    WINDOW * main_window_;
    WINDOW * command_window_;
    WINDOW * status_window_;

    std::shared_ptr<FILE> command_pipe_;

    enum Mode
    {
        NONE,
        NODE_LIST,
        TOPIC_LIST,
        SERVICE_LIST,
        NODE_INFO,
        TOPIC_INFO,
        SERVICE_INFO
    };

    Mode mode_;


    void initNcurses();

    void writeCommand(const std::string &command);
    void writeStatus(const std::string &status);
    void writeOutput();
    void updateOutput();

    void runCommand(const std::string &command);
    void parseInput(char input, std::string &command);

    void highlightNextRow();
    void highlightPreviousRow();

public:
    NcursesNode();
    virtual ~NcursesNode();

    void run();
};

#endif /* NCURSES_NODE_H */
