#include "arm_state.hpp"
#include "json.hpp"
#include "utils.hpp"
//Should these be .lcm?
#include "rover_msgs/ArmPosition.hpp"
#include "rover_msgs/TargetOrientation.hpp"
#include "kinematics.hpp"
#include <iostream>
#include <Eigen/Dense>
#include<string>
#include<vector>
using namespace Eigen;
using namespace std;

MRoverArm::MRoverArm(json config, lcm::LCM lcm) : config(config), lcm_(lcm) {
    json geom = read_geometry_from_JSON();
    ArmState state = ArmState(geom);
    KinematicSolver solver = KinematicsSolver(state, lcm);
    MotionPlanner motion_planner = MotionPlanner(state, lcm, solver);
    // current_spline = [];  // list
    int spline_t = 0;
    bool done_previewing = False;
    bool enable_execute = False;
    bool sim_mode = True;
    bool ik_enabled = False;
}

void MRoverArm::arm_position_callback(string channel, ArmPosition msg){
    /*
        Handler for angles
        Triggers forward kinematics
    */  
    if (ik_enabled && !enable_execute){
        return;
    }

    ArmPosition arm_position = ArmPosition.decode(msg)
    if (channel == "/arm_position") {
        state.set_angles(arm_position);
        solver.FK(state);
        publish_transforms(state);
    }
}

void MRoverArm::publish_config(Vector6d config, string channel){
        ArmPosition arm_position = ArmPosition();
        arm_position.joint_a = config[0];
        arm_position.joint_b = config[1];
        arm_position.joint_c = config[2];
        arm_position.joint_d = config[3];
        arm_position.joint_e = config[4];
        arm_position.joint_f = config[5];

        lcm_.publish(channel, arm_position.encode());
}

void MRoverArm::publish_transforms(ArmState state){
        tm = FKTransform();
        tm.transform_a = state.get_joint_transform('joint_a');
        tm.transform_b = state.get_joint_transform('joint_b');
        tm.transform_c = state.get_joint_transform('joint_c');
        tm.transform_d = state.get_joint_transform('joint_d');;
        tm.transform_e = state.get_joint_transform('joint_e');
        tm.transform_f = state.get_joint_transform('joint_f');
        lcm_.publish('/fk_transform', tm.encode());
}
      

void MRoverArm::target_orientation_callback(string channel, TargetOrientation msg){
    cout << "target orientation callback";
    TargetOrientation point_msg = msg;
    enable_execute = false;
    //this statement replaced logger
    cout << "Got a target point." << endl;

    cout << "alpha beta gamma" << endl << endl;
    cout << point_msg.alpha << " , " << point_msg.beta << " , " << point_msg.gamma << endl << endl;
    bool use_orientation = point_msg.use_orientation;
    cout << use_orientation << endl; 

    double point[6] = {point_msg.x, point_msg.y, point_msg.z, point_msg.alpha, point_msg.beta, point_msg.gamma};

    bool success = false;
    //I'm not sure about the syntax for the vector pair here
    pair<vector<double> joint_angles, success> = solver.IK(point, false, use_orientation);
    //idk the syntax for debug here
    DebugMessage ik_message = DebugMessage();
    ik_message.isError = false;

    for(int i = 0; i<5; ++i){
        if(success){
            ik_message.message = "Solved IK";
            break;
        }
        cout << "Attempting new IK solution..." << endl << i << endl;
        pair<vector<double> joint_angles, success> = solver.IK(point, true, use_orientation);
    }
    if(!success){
        ik_message.message = "No IK Solution";
        cout << "NO IK SOLUTION FOUND, using closest configuration..." << endl;
    }

    //What is the equivalent of .encode in c++ and do we need it
    lcm_.publish("/debugMessage", ik_message.encode());
    if(!success){
        return;
    }

    publish_transforms(state);
    //Is it curly brackets for a vector?
    Vector6d goal(joint_angles["joint_a"], joint_angles["joint_b"], joint_angles["joint_c"], joint_angles["joint_d"], joint_angles["joint_e"],joint_angles["joint_f"]);
    plan_path(goal);
}

void MRoverArm::target_angles_callback(string channel, TargetAngles msg){
    enable_execute = false;
    //not sure of data type
    TargetAngles target_angles = msg;
    //should this be a double array or a vector6d
    Vector6d goal(target_angles.joint_a,
                target_angles.joint_b,
                target_angles.joint_c,
                target_angles.joint_d,
                target_angles.joint_e,
                target_angles.joint_f);
    for(int i = 0; i<6; i++){
        cout << goal[i] << "/n";
    }
    cout << endl;
    //why is the color different here for plan_path
    plan_path(goal); 
}

void MRoverArm::plan_path(Vector6d goal){
    cout << "goal" << endl;
    for(int i = 0; i<goal.size(); i++){
        cout << goal[i] << "/n";
    }
    cout << endl << "start" << endl;
    //What is state.angles, is it vector6d?
    map<string, double> joint_angles = state.get_joint_angles();
    for(int i = 0; i<state.angles.size(); i++){
        cout << state.angles[i] << "/n";
    }
    cout << endl;    
    //Again, idk the type for debugmessage
    DebugMessage path_message = DebugMessage(); 
    path_message.isError = false;
    //idk this data type
    vector<tk::spline> path_spline = motion_planner.rrt_connect(goal);
    if(path_spline){
        spline_t = 0;
        cout << "planned path" << endl;
        path_message.message = "Planned Path"; 
    }
    else{
        cout << "No path found" << endl;
        path_message.message = "No path found";
    }
    lcm_.publish("/debugMessage", path_message.encode());
}

void MRoverArm::motion_execute_callback(string channel, MotionExecute msg){
    MotionExecute motion_execute_msg = msg;

    bool preview = motion_execute_msg.preview;
    if(preview){
        enable_execute = false;
        //what is preview
        preview();
    }
    else{
        enable_execute = true;
    }
}

void MRoverArm::simulation_mode_callback(string channel, SimulationMode msg){
    SimulationMode simulation_mode_msg = msg;

    bool sim_mode = simulation_mode_msg.sim_mode;
    publish_transforms(state);
}

void MRoverArm::cartesian_control_callback(string channel, IkArmControl msg){
    if(enable_execute){
        return;
    }

    IkArmControl cart_msg = msg;
    double delta[3] = {cart_msg.deltaX, cart_msg.deltaY, cart_msg.deltaZ};
    //idk if this line is right down here
    pair<vector<double> joint_angles, bool is_safe> = solver.IK_delta(delta, 3);
    
    if(is_safe){
        ArmPosition arm_position = ArmPosition();
        arm_position.joint_a = joint_angles["joint_a"];
        arm_position.joint_b = joint_angles["joint_b"];
        arm_position.joint_c = joint_angles["joint_c"];
        arm_position.joint_d = joint_angles["joint_d"];
        arm_position.joint_e = joint_angles["joint_e"];
        arm_position.joint_f = joint_angles["joint_f"];
        //what is set_angles
        state.set_angles(arm_position);
        solver.FK(state);
        publish_transforms(state);
        //again, running into the issue of encode(), should we even have it there
        if(sim_mode){
            cout << "Printing sim_mode" << endl;
            lcm_.publish("/arm_position", arm_position.encode());
        }
        else{
            cout << "Printing" << endl;
            lcm_.publish("/ik_ra_control", arm_position.encode());
        }
    }


}
