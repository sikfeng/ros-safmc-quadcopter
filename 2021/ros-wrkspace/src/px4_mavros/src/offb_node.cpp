#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <std_srvs/Trigger.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

mavros_msgs::ExtendedState current_extended_state;
void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    current_extended_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber extended_sub = nh.subscribe("mavros/extended_state", 10, extended_state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient eraser_dropper_client = nh.serviceClient<std_srvs::Trigger>("drop_eraser");


    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    std_srvs::Trigger eraser_drop_cmd;

    ros::Time last_request = ros::Time::now();

    for (int i = 0; ros::ok(); ++i) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

	if (i == 10) {
    eraser_dropper_client.call(eraser_drop_cmd);
    if (!eraser_drop_cmd.response.success) {
      ROS_WARN("eraser drop cmd failed");
    }
	}
	if (i == 100) {
    eraser_dropper_client.call(eraser_drop_cmd);
    if (!eraser_drop_cmd.response.success) {
      ROS_WARN("eraser drop cmd failed");
    }
	}
	if (i == 200) {
    eraser_dropper_client.call(eraser_drop_cmd);
    if (!eraser_drop_cmd.response.success) {
      ROS_WARN("eraser drop cmd failed");
    }
	}
	if (i == 300) {
    eraser_dropper_client.call(eraser_drop_cmd);
    if (!eraser_drop_cmd.response.success) {
      ROS_WARN("eraser drop cmd failed");
    }
	}

	if (i == 300) {
            pose.pose.position.x = 1;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1;
	}
	if (i == 600) {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0;
	}

	if (current_extended_state.landed_state == current_extended_state.LANDED_STATE_ON_GROUND) {
	    //break;
	}
	if (i > 216000) {
	    break;
	}

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
