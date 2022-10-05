#include "ih2_azzurra_command.h"

inline int clamp(int val, int low_l, int high_l){ 
    return ( (val < low_l) ? low_l : ( high_l< val ) ? high_l : val ); }


IH2Command::IH2Command(){}

IH2Command::~IH2Command(){}


void IH2Command::sendPositionCommand(std::vector<std::string> joint_names, std::vector<trajectory_msgs::JointTrajectoryPoint> pos)  
{   
    this->ag_status_ = 102;
    this->ag_grasp_= "RND";

    std::vector<std::string> all_joints = {"THU_A", "THU_F", "IND_F", "MID_F", "RIL_F"};

    int jn_c=0;

    for (auto &joint : joint_names) 
    {
        for (int aj_c : {0, 1, 2, 3, 4})
        {
            if (joint.compare(all_joints[aj_c])==0)  // if joint == all_joints(aj)
            {
                int motor = aj_c;
                int position = static_cast<int> (pos[jn_c].positions[0]);
                ROS_INFO("motore %i, posizione %i", motor, position);

                //std::string command = {(char) 0x44, (char) motor, (char)position};
                //ROS_INFO("|%02x %02x %02x|", command[0], command[1], command[2]);

                std::string command = {(char) 0x44, (char) aj_c, (char) pos[jn_c].positions[0]};
                this->ih2_serial1_->Write(command);
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }
        jn_c++;
    }


}


void IH2Command::sendAutoGraspCommand(std::string graspType, int graspStep, int graspForce)
{
    ROS_INFO("ag_status_ pre %i", this->ag_status_);
    ROS_INFO("ag_grasp_ pre %s", this->ag_grasp_.c_str());
    ROS_INFO("grasp_step_pre pre %i", graspStep);

    // Manage GraspTypeSettings
    std::vector<std::string> all_grasps = {"RLX", "LAT", "PIN", "TRI", "CYL"};

    int ag_c = 0;
    int ag_existence = 0;
    char graspTypeChar;

    for (auto &grasp : all_grasps)
    {
        if (grasp.compare(graspType)==0)
        {
            graspTypeChar = (char) ag_c;
            ag_existence = 1;

            if (graspType.compare( this->ag_grasp_ )!=0){   // se sto cambiando il tipo di autograsp rispetto al precedente
                std::string command = {(char) 0x4E, (char) 0x00, graspTypeChar, (char) graspForce, (char) 0x4E};
                ROS_INFO("S: 00, T: %i, F %i", ag_c, graspForce);          // prima mando in preshape
                this->ih2_serial1_->Write(command);
                this->ag_grasp_ = graspType;
                this->ag_status_= 0; 
                ROS_INFO("ag_status_ preshape %i", this->ag_status_);

                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }

        ag_c++;

    }

    if (ag_existence == 0){ //se viene scritta un'etichetta che non rappresenta nessun grasp             
        ROS_INFO("This autograsp does not exist");
        this->ag_status_ = 102;
        return;
    }


    // Manage GraspStepSettings
    graspStep = clamp(graspStep, 0, 100);
    ROS_INFO("grasp_step_ clamped %i", graspStep);

    char graspStepChar;

    if (graspStep == 0)       //se si comanda una preshape
    {
        graspStepChar = (char) 0x00; //metto in preshape
        this->ag_status_=0;
    }
    else
    {   
        ROS_INFO("ag_status_ preswitch %i", this->ag_status_);
        int remap_result=0;
        switch (this->ag_status_)
        {
            case 0:     //se sono in preshape
            {
                graspStep = graspStep - 1;
                remap_result = remap_lohi(graspStep, 120, 240);
                graspStepChar = (unsigned char) (remap_result);      //chiudo
                this->ag_status_= 101;                                           //e segnalo che ho chiuso
                break;
            }
            case 101:   //se ho chiuso
            {
                graspStep = graspStep-1; //da 0 a 99
                remap_result = remap_lohi(graspStep, 30, 90);        //apro di un tot
                graspStepChar = (unsigned char) (remap_result); 
                this->ag_status_= graspStep+1;                                     //e segnalo di quanto ho riaperto (1 to 100)
                break;
            }
            default:    //se ho già riaperto di un tot,  
            {
                if (graspStep < this->ag_status_)
                {
                    graspStep = graspStep-1; //da 0 a 99            
                    remap_result = remap_lohi(graspStep, 30, 90);    //riapro di un altro tot
                    graspStepChar = (unsigned char) (remap_result); 
                    this->ag_status_= graspStep+1;                                 //e segnalo di quanto ho riaperto (1 to 100)
                }
                else                                 
                {
                    graspStep = graspStep - 1;
                    remap_result = remap_lohi(graspStep, 120, 240);  //o richiudo
                    graspStepChar = (unsigned char) (remap_result); 
                    this->ag_status_= 101;                                         //e segnalo che ho chiuso
                }
                break; 
            }
            

        }
        ROS_INFO("grasp_step_ remapped %i", remap_result);
    }


    // Manage graspForceSettings
    graspForce = clamp(graspForce, 0, 255);


    // Send Command
    std::string command = {(char) 0x4E, graspStepChar, graspTypeChar, (char) graspForce, (char) 0x4E};

    ROS_INFO("S: %i, T: %i, F %i", (unsigned int) graspStepChar, (unsigned int) graspTypeChar, graspForce); 
    ROS_INFO("%s", command.c_str());
    ROS_INFO("ag_status_ post %i", this->ag_status_);
    ROS_INFO("ag_grasp_ post %s", this->ag_grasp_.c_str());
    this->ih2_serial1_->Write(command);

}


void IH2Command::enableStreaming(int streaming_mode){
    if (this->streaming_status_ == true)
    {
        this->streaming_status_ = false;
        this->ih2_serial1_->WriteByte((char) 0x47);//close streaming
        ROS_INFO("Streaming %i disabled", streaming_mode);
    }
    else
    {
        this->streaming_status_ = true;
        std::string command = {(char) 0x43, (char) streaming_mode};
        this->ih2_serial1_->Write(command);       //open streaming
        ROS_INFO("Streaming %i enabled", streaming_mode);

    }
}





int IH2Command::remap_lohi(int val, int low_l, int high_l){ 
    float val_f = val;
    ROS_INFO("val_f %f", val_f);
    int remapped = std::round(  (val_f /99) * (high_l-low_l) );
    ROS_INFO("remapped %i", remapped);

    return low_l + remapped;
}






/* 
void IH2Command::setSerial(std::shared_ptr<LibSerial::SerialPort> serial_port)
{
    this->ih2_serial1_ = serial_port;
    //this->ih2_serial1_->WriteByte((char) 0x42); //here it works

    ROS_INFO("setport");
}  */
