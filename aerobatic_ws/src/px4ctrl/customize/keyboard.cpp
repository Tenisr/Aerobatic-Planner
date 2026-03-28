#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <geometry_msgs/PointStamped.h>

struct termios old_tio;

enum Mode
{
    MANUAL,
    NAVIGATION
};

void restore_terminal_settings()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

void quit(int sig)
{
    (void)sig;
    restore_terminal_settings();
    ros::shutdown();
    exit(0);
}

void configure_terminal()
{
    struct termios new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void print_instructions(float x=0.0, float y=0.0, float z=1.0, float yaw=0.0, Mode mode=MANUAL)
{
    std::cout << "\033[2J\033[1;1H";
    std::cout << "---------------------------" << std::endl;
    std::cout << "Keyboard Control for Drone" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "w/s : move forward/backward (x-axis)" << std::endl;
    std::cout << "a/d : move left/right (y-axis)" << std::endl;
    std::cout << "j/k : move up/down (z-axis)" << std::endl;
    std::cout << "q/e : yaw left/right" << std::endl;
    std::cout << "l   : stop" << std::endl;
    std::cout << "t   : control mode   :" << (mode == MANUAL ? "MANUAL" : "NAVIGATION") << std::endl;
    std::cout << "x   : quit" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Current Command:" << std::endl;
    std::cout << "Position -> x: " << x << ", y: " << y << ", z: " << z << ", Yaw -> " << yaw <<std::endl;
    std::cout << "---------------------------" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_commander");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    ros::Publisher trigger_pub = nh.advertise<geometry_msgs::PointStamped>("/trigger", 10);

    configure_terminal();
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    signal(SIGINT, quit);

    ros::Rate loop_rate(5); 

    quadrotor_msgs::PositionCommand cmd;
    cmd.position.x = 0.0;
    cmd.position.y = 0.0;
    cmd.position.z = 1.0;
    cmd.yaw = 0.0;

    geometry_msgs::PointStamped trigger;

    // cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    char intention;
    Mode mode = MANUAL;

    std::vector<char> key_values = {'w', 'a', 's', 'd', 'j', 'k', 'q', 'e','t','x','l'};

    bool trigger_sent = false;

    while (ros::ok())
    {
        print_instructions(cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw, mode);

        char c = getchar();

        if (c != EOF && std::find(key_values.begin(), key_values.end(), c) != key_values.end())
        {
            intention = c;
        }
        switch (intention)
        {
            case 'w':
                cmd.position.x += 0.1;
                break;
            case 's':
                cmd.position.x -= 0.1;
                break;
            case 'a':
                cmd.position.y += 0.1;
                break;
            case 'd':
                cmd.position.y -= 0.1;
                break;
            case 'j':
                cmd.position.z += 0.1;
                break;
            case 'k':
                cmd.position.z -= 0.1;
                break;
            case 'e':
                cmd.yaw -= 0.1;
                cmd.yaw_dir.x = cos(cmd.yaw);
                cmd.yaw_dir.y = sin(cmd.yaw);
                cmd.yaw_dir.z = 0.0;
                break;
            case 'q':
                cmd.yaw += 0.1;
                cmd.yaw_dir.x = cos(cmd.yaw);
                cmd.yaw_dir.y = sin(cmd.yaw);
                cmd.yaw_dir.z = 0.0;
                break;
            case 't':
                if(mode == MANUAL){
                    mode = NAVIGATION;
                    trigger_sent = false;
                }else{
                    mode = MANUAL;
                }
                intention = 'l';
                break;
            case 'x':
                quit(0);
                break;
        }

        if(mode == MANUAL){
            cmd_pub.publish(cmd);
        }else if(mode == NAVIGATION){
            if(!trigger_sent){
                trigger_pub.publish(trigger);
                trigger_sent = true;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    restore_terminal_settings();
    return 0;
}