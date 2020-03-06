#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

RobotPose robotPose(0,0,0);
int minbox;

float Nx, Ny, Oyaw;

float new_pos[5][3]; //std::vector<std::vector<float>> new_pos;
float delcoords[5][3];

void New_position (float x, float y, float yaw)
{
    float Ayaw = yaw*180 / M_PI;
    float PAyaw, OAyaw;
    int distance_away = 0.4;
    if(Ayaw < 0 && Ayaw != -180)
    {
        PAyaw = 180 + (180 - abs(Ayaw));
        Nx = x + distance_away*cos(180 - abs(PAyaw));
        Ny = y + distance_away*sin(180 - abs(PAyaw));
        Oyaw = 180 - abs(Ayaw);
    }
    else if(Ayaw == -180)
    {
        PAyaw = 180;
        Nx = x + distance_away*cos(180 - abs(PAyaw));
        Ny = y + distance_away*sin(180 - abs(PAyaw));
        OAyaw = 0;
    }
    else
    {
        Nx = x + distance_away*cos(180 - abs(Ayaw));
        Ny = y + distance_away*sin(180 - abs(Ayaw));
        OAyaw = (180 - Ayaw)*(-1);
    }
    Oyaw = OAyaw*M_PI / 180;
    return;
}

void nextbox() { //return minbox from 0 to 4
    float dist;
    float minboxdist = 100.00;
    minbox = 6;

    for(int i = 0; i <= 4; ++i){
        dist = sqrt(pow((delcoords[i][0] - robotPose.x),2) + pow((delcoords[i][1] - robotPose.y),2));
        if (dist < minboxdist){
            minbox = i;
            minboxdist = dist;
        }    
    }

    if (minbox == 6){ // should only enter when done traversing all 5
        minbox = 1;
    }

    delcoords[minbox][0] = 1000.00;
    delcoords[minbox][1] = 1000.00;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    //************************************************ RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " << boxes.coords[i][2] << std::endl;
        New_position (boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
        new_pos[i][0] = Nx;
        new_pos[i][1] = Ny;
        new_pos[i][2] = Oyaw;
        delcoords[i][0] = Nx;
        delcoords[i][1] = Ny;
        delcoords[i][2] = Oyaw;
    }

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        

        //sarahs stuff
        bool sarahsdone=true;
        if(sarahsdone){
            Navigation::moveToGoal(new_pos[minbox][0],new_pos[minbox][1],new_pos[minbox][2]);
            ROS_INFO("%f , %f , %f", new_pos[minbox][0],new_pos[minbox][1],new_pos[minbox][2]);
        }
        
        imagePipeline.getTemplateID(boxes);

        ros::Duration(0.01).sleep();
    }
    return 0;
}
