
#include "uwb-test.hpp"
UWB_Tag_Sim::UWB_Tag_Sim():Node("UWB_Tag_Simulation_Node"){
    std::random_device rd;
    tandomGenerator = std::default_random_engine(rd());
    this->declare_parameter("tag_name","Drone");//default to drone
    tag_name = this->get_parameter("tag_name").as_string();
    std::string world_topic = "/world/car_world/pose/info";//this can be changed check gz topic -l
    if(!subscription_to_world.Subscribe(world_topic, &UWB_Tag_Sim::gazeboworldstats_executor_callback,this)){
        RCLCPP_ERROR(this->get_logger(),"Failed to subscribe to the world file!");
    }
    uwb_ranges_array_publisher = this->create_publisher<mcu_msgs::msg::UWBRanging>("/"+tag_name+"/ranging",10);
    using namespace std::chrono_literals;
        wall_timer = this->create_wall_timer(30ms, std::bind(&UWB_Tag_Sim::rostwo_executor_callback, this));
}
void UWB_Tag_Sim::gazeboworldstats_executor_callback(const gz::msgs::Pose_V &msg){
    std::lock_guard<std::mutex>lock(msg_mutex);
    latest_position_of_objects = msg;
    has_new_data = true;//checks if the gazebo world has new data to publish
}
void UWB_Tag_Sim::rostwo_executor_callback(){
    gz::msgs::Pose_V current_data;//this will be updated on each call
    {
        std::lock_guard<std::mutex>lock(msg_mutex);
        if(!has_new_data) return; //no changes if false
        current_data = latest_position_of_objects;
        has_new_data = false;
    }
    gz::msgs::Pose my_position;//can be position of either tag or anchor
    bool found_current_position=false;
    //Pose v is an array of pose information however, each pose information is stuck to a tag
    //whatever tag you want to find you need to change the tag_name
    for(int i =0; i < current_data.pose_size(); i++){
        if(current_data.pose(i).name() == tag_name){
            my_position = current_data.pose(i);
            found_current_position =true;
            break;
        }
    }
    if(!found_current_position) return;
    mcu_msgs::msg::UWBRanging uwb_ranges_array;

    for(int i =0; i < current_data.pose_size();i++){
        const auto &entity = current_data.pose(i);
        std::string name  = entity.name();

        if((name.find("anchor") !=std::string::npos)){
            double dx = entity.position().x() - my_position.position().x();
            double dy = entity.position().y() - my_position.position().y();
            double dz = entity.position().z() - my_position.position().z();
            double distance = std::sqrt((dx*dx)+(dy*dy)+(dz*dz));

            std::normal_distribution<double> distribution_normal(0., 0.1);
            double simulated_distance = distance + distribution_normal(tandomGenerator);
            mcu_msgs::msg::UWBRange range_info;

            if(tag_name=="Drone") range_info.tag_id=20;
            if(tag_name=="MiniBot") range_info.tag_id = 21;
            rclcpp::Time now = this->now();
            if(name.find("anchor_10")!=std::string::npos){
                range_info.distance = simulated_distance;//might be published in meters need to convert to centimeters
                range_info.anchor_id = 10;
                range_info.signal_strength = 0.0;//not sure what to put for now
                range_info.clock_offset = 0.0;//not sure what to put for now
                range_info.tx_timestamp = now.seconds();//not sure
                range_info.rx_timestamp = now.seconds();//not sure
                range_info.valid = true;
                range_info.error_code =0;
                uwb_ranges_array.ranges.push_back(range_info);

            }else if(name.find("anchor_11")!=std::string::npos){
                range_info.distance = simulated_distance;
                range_info.anchor_id = 11;
                range_info.signal_strength = 0.0;//not sure what to put for now
                range_info.clock_offset = 0.0;//not sure what to put for now
                range_info.tx_timestamp = now.seconds();//not sure
                range_info.rx_timestamp = now.seconds();//not sure
                range_info.valid = true;
                range_info.error_code =0;
                uwb_ranges_array.ranges.push_back(range_info);

            }else if(name.find("moving_anchor12")!=std::string::npos){
                range_info.distance = simulated_distance;
                range_info.anchor_id = 12;
                range_info.signal_strength = 0.0;//not sure what to put for now
                range_info.clock_offset = 0.0;//not sure what to put for now
                range_info.tx_timestamp = now.seconds();//not sure
                range_info.rx_timestamp = now.seconds();//not sure
                range_info.valid = true;
                range_info.error_code =0;
                uwb_ranges_array.ranges.push_back(range_info);    
            }
        }
    }
    // uwb_ranges_array_publisher->publish(uwb_ranges_array);
    // At the very end of rostwo_executor_callback 
    if (!uwb_ranges_array.ranges.empty()) {
        uwb_ranges_array_publisher->publish(uwb_ranges_array);
    }
}
int main(int argc, char **argv){
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWB_Tag_Sim>());
    rclcpp::shutdown();

    return 0;
}