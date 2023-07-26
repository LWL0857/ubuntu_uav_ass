#include "uav_base/uav_base.h"
extern  MotorPwm pwm_data_;
extern   RcData rc_data_;


void UavBase::imu_publisher()
{
    RCLCPP_INFO(this->get_logger(), "Imu Data Publish.");
    // 封装IMU的话题消息
    auto imu_msg = uav_msgs::msg::Imu();

    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = this->get_clock()->now();

    imu_msg.acc_x = imu_data_.acc_x;
    imu_msg.acc_y = imu_data_.acc_y;
    imu_msg.acc_z = imu_data_.acc_z;

    imu_msg.gyro_x = imu_data_.gyro_x;
    imu_msg.gyro_y = imu_data_.gyro_y;
    imu_msg.gyro_z = imu_data_.gyro_z;

    // 发布IMU话题
    imu_publisher_->publish(imu_msg);
}

void UavBase::status_publisher()
{
    RCLCPP_INFO(this->get_logger(), "STATUS Data Publish.");
    auto status_msg=uav_msgs::msg::UavStatus();
    status_msg.header.frame_id = "status_link";
    status_msg.header.stamp = this->get_clock()->now();
    status_msg.ahrs_eular_x = uav_status_.ahrsEular_x;
    status_msg.ahrs_eular_y = uav_status_.ahrsEular_y;
    status_msg.ahrs_eular_z = uav_status_.ahrsEular_z;
    status_msg.height = uav_status_.height;
    status_msg.battery_voltage = uav_status_.battery_voltage;
    status_msg.mode = uav_status_.mode;
    status_msg.lock = uav_status_.lock;

    status_publisher_->publish(status_msg);
}
void UavBase::uwb_publisher()
{
    RCLCPP_INFO(this->get_logger(), "UWB Data Publish.");
    //封装uwb的话题消息
    auto uwb_msg=uav_msgs::msg::UavUwb();
    uwb_msg.header.frame_id = "uwb_link";
    uwb_msg.header.stamp = this->get_clock()->now();
    uwb_msg.pos_x=uwb_data_.pos_x;
    uwb_msg.pos_y=uwb_data_.pos_y;
    uwb_msg.pos_z=uwb_data_.pos_z;

    uwb_publisher_->publish(uwb_msg);
}
void UavBase::mag_publisher()
{
    RCLCPP_INFO(this->get_logger(), "MAG Data Publish.");

    auto mag_msg=uav_msgs::msg::Mag();
    mag_msg.header.frame_id = "mag_link";
    mag_msg.header.stamp = this->get_clock()->now();

    //auto now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    mag_msg.magraw_x=mag_data_.magraw_x;
    mag_msg.magraw_y=mag_data_.magraw_y;
    mag_msg.magraw_z=mag_data_.magraw_z;

    mag_publisher_->publish(mag_msg);

}

void UavBase::motorpwm_publisher()
{
    RCLCPP_INFO(this->get_logger(), "MOTOR_PWM Data Publish.");
    auto pwm_msg=uav_msgs::msg::MotorPwm();
    pwm_msg.header.frame_id = "pwm_link";
    pwm_msg.header.stamp = this->get_clock()->now();
    pwm_msg.pwm_0=pwm_data_.pwm_0;
    pwm_msg.pwm_1=pwm_data_.pwm_1;
    pwm_msg.pwm_2=pwm_data_.pwm_2;
    pwm_msg.pwm_3=pwm_data_.pwm_3;

    motorpwm_publisher_->publish(pwm_msg);


}
void UavBase::rc_publisher()
{
    RCLCPP_INFO(this->get_logger(), "RC Data Publish.");

    auto rc_msg = uav_msgs::msg::Rc();
    rc_msg.header.frame_id = "pwm_link";
    rc_msg.header.stamp = this->get_clock()->now();
    rc_msg.ppm_0 = rc_data_.ppm_0;
    rc_msg.ppm_1 = rc_data_.ppm_1;
    rc_msg.ppm_2 = rc_data_.ppm_2;
    rc_msg.ppm_3 = rc_data_.ppm_3;
    rc_msg.ppm_4 = rc_data_.ppm_4;
    rc_msg.ppm_5 = rc_data_.ppm_5;
    rc_msg.ppm_6 = rc_data_.ppm_6;
    rc_msg.ppm_7 = rc_data_.ppm_7;

    rc_publisher_->publish(rc_msg);
}
void UavBase::flow_publisher()
{
    RCLCPP_INFO(this->get_logger(), "FLOW Data Publish.");
    auto flow_msg = uav_msgs::msg::Flow();
    
    flow_msg.header.frame_id = "flow_link";
    flow_msg.header.stamp = this->get_clock()->now();
    
    flow_msg.pos_x=flow_data_.pos_x;
    flow_msg.pos_y=flow_data_.pos_y;
    flow_msg.speed_x=flow_data_.speed_x;
    flow_msg.speed_y=flow_data_.speed_y;

    flow_publisher_->publish(flow_msg);
}


UavBase::UavBase(std::string nodeName) : Node(nodeName)
{
    // 加载参数
    std::string port_name="ttyS3";    
    this->declare_parameter("port_name");           //声明及获取串口号参数
    this->get_parameter_or<std::string>("port_name", port_name, "ttyS3");

    std::string mocap_sub_name="/vrpn_mocap/RigidBody/pose";
    this->declare_parameter("mocap_sub_name");
    this->get_parameter_or<std::string>("mocap_sub_name",mocap_sub_name,"/vrpn_mocap/RigidBody/pose");

    this->declare_parameter("use_imu");            
    this->declare_parameter("use_uwb");
    this->declare_parameter("use_mocap");

    this->get_parameter_or<bool>("use_uwb",use_uwb_,false);
    this->get_parameter_or<bool>("use_mocap",use_mocap_,false);
    this->get_parameter_or<bool>("use_imu", use_imu_, false);


    status_publisher_ = this->create_publisher<uav_msgs::msg::UavStatus>("uav_status", 0.1);
    uwb_publisher_= this->create_publisher<uav_msgs::msg::UavUwb>("uwb", 0.1);
    imu_publisher_= this->create_publisher<uav_msgs::msg::Imu>("imu", 0.1);
    mag_publisher_ = this->create_publisher<uav_msgs::msg::Mag>("mag", 0.1);
    rc_publisher_= this->create_publisher<uav_msgs::msg::Rc>("rc", 0.1);
    motorpwm_publisher_ = this->create_publisher<uav_msgs::msg::MotorPwm>("motorpwm", 0.1);  
    flow_publisher_ = this->create_publisher<uav_msgs::msg::Flow>("flow", 0.1);
    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 控制器与扩展驱动板卡的串口配置与通信
    try
    {
        serial_.setPort("/dev/" + port_name);                            //选择要开启的串口号
        serial_.setBaudrate(115200);                                     //设置波特率
        serial::Timeout timeOut = serial::Timeout::simpleTimeout(2000);  //超时等待
        serial_.setTimeout(timeOut);                                     
        serial_.open();                                                  //开启串口
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "uav can not open serial port,Please check the serial port cable! "); //如果开启串口失败，打印错误信息
    }

    // 如果串口打开，则驱动读取数据的线程
    if (serial_.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "uav serial port opened"); //串口开启成功提示

        // 启动一个新线程读取并处理串口数据
        read_data_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&UavBase::readRawData, this)));
    }

    if(use_imu_)
    {
        // 创建IMU的话题发布者
       // imu_publisher_    = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

        // IMU初始化标定
       /*
        if(imu_calibration())
        {
            usleep(500000);    //确保标定完成
            RCLCPP_INFO(this->get_logger(), "IMU calibration ok.");
        }
        */
    }

    if(use_uwb_)
    {
        //创建uwB的话题发布者
        uwb_publisher_ =this->create_publisher<uav_msgs::msg::UavUwb>("uav_uwb",1);
    }
    if(use_flow_)
    {
        //创建uwB的话题发布者
        flow_publisher_ =this->create_publisher<uav_msgs::msg::Flow >("flow",1);
    }

    if(use_mocap_)
    {
        mocap_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(mocap_sub_name, 10,std::bind(&UavBase::mocap_pos_callback, this, _1));

    }
   
    RCLCPP_INFO(this->get_logger(), "Uav Start, enjoy it.");
}

UavBase::~UavBase()
{
    serial_.close();
}

void UavBase::readRawData()
{
    uint8_t rx_data = 0;
    // uint8_t rx_data1= 0;
    DataFrame frame;

    while (rclcpp::ok()) 
    {
        // 读取一个字节数据，寻找帧头
        auto len = serial_.read(&rx_data, 1);
        if (len < 1)
            continue;
        
        // 发现帧头后开始处理数据帧
        if(rx_data == 0x55)
        {
           /*  auto length_num=serial_.read(&rx_data1,1)
            switch(length_num)
            {  case FRAME_ID_STATUS:
                    serial_.read(&frame.id, 16);
                    break;
                case FRAME_ID_ACCELERATION:
                    break;
                case FRAME_ID_ANGULAR:
                     break;
                case FRAME_ID_EULER:
                      break;
                case FRAME_ID_SENSOR:
                    break;
                case FRAME_ID_UWB:
                default:
                    break;
            } */
            // 读取完整的数据帧
            serial_.read(&frame.id, 32);
            //ID,开始的32个byte.7*4+4=32

            // 判断帧尾是否正确
            if(frame.tail != 0xbb)
            {
                RCLCPP_WARN(this->get_logger(), "Data frame tail error!"); 
                printf("Frame raw data[Error]: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
                        frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2], 
                        frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);
                continue;
            }

            frame.header = 0x55;
            
            // 帧校验
            if(checkDataFrame(frame))
            {
                // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
                //         frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2], 
                //         frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);

                //处理帧数据
                processDataFrame(frame);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Data frame check failed!"); 
            }
        }
    }
}

bool UavBase::checkDataFrame(DataFrame &frame)
{    
    int len=frame.length;
    uint8_t sum=0;
    for(int i=0;i<len;i++)
    {   
        sum+=frame.data[i];

    }
    if((sum & 0xff)==frame.check)
        return true;
    else
        return false;
    /*
    
    if(((frame.data[0] + frame.data[1] + frame.data[2] + 
        frame.data[3] + frame.data[4] + frame.data[5]) & 0xff) == frame.check)
        return true;
    else
        return false;
    */
}

void UavBase::processDataFrame(DataFrame &frame)
{
    // 根据数据帧的ID，对应处理帧数据
    switch(frame.id)
    {
    case FRAME_ID_STATUS:
        processStatusData(frame);
        break;
    case FRAME_ID_IMU:
        processImuData(frame);
        break;
    case FRAME_ID_MAG:
        processMagData(frame);
        break;
    case FRAME_ID_UWB:
        processUwbData(frame);
        break; 
    case FRAME_ID_RC:
        processRcData(frame);
        break;
    case FRAME_ID_PWM:
        processMotorPWMData(frame);
        break;
        
    case FRAME_ID_FLOW:
        processFlowData(frame);
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Frame ID Error[%d]", frame.id);
        break;
    }
}

void UavBase::processStatusData(DataFrame &frame)
{
    RCLCPP_INFO(this->get_logger(), "Process status data");

    DataFloat ahrsEular_x,ahrsEular_y,ahrsEular_z,height,battery_voltage;

    memcpy(&ahrsEular_x.data,&frame.data[0],4);
    memcpy(&ahrsEular_y.data,&frame.data[4],4);
    memcpy(&ahrsEular_z.data,&frame.data[8],4);
    memcpy(&height.data,&frame.data[12],4);
    memcpy(&battery_voltage.data,&frame.data[16],4);

    uav_status_.ahrsEular_x=ahrsEular_x.f;
    uav_status_.ahrsEular_y=ahrsEular_y.f;
    uav_status_.ahrsEular_z=ahrsEular_z.f;
    uav_status_.height=height.f;
    uav_status_.battery_voltage=battery_voltage.f;

    memcpy(&uav_status_.mode,&frame.data[20],4);
    memcpy(&uav_status_.lock,&frame.data[21],4);
   
    status_publisher();    
}

void UavBase::processImuData(DataFrame &frame)
{
    RCLCPP_INFO(this->get_logger(), "Process IMU data");
    DataFloat acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z;
    memcpy(&acc_x.data, &frame.data[0], 4);
    memcpy(&acc_y.data, &frame.data[4], 4);
    memcpy(&acc_z.data, &frame.data[8], 4);
    memcpy(&gyro_x.data, &frame.data[12], 4);
    memcpy(&gyro_y.data, &frame.data[16], 4);
    memcpy(&gyro_z.data, &frame.data[20], 4);
    imu_data_.acc_x=acc_x.f;
    imu_data_.acc_y=acc_y.f;
    imu_data_.acc_z=acc_z.f;
    imu_publisher();
}
void UavBase::processMagData(DataFrame &frame)
{
    RCLCPP_INFO(this->get_logger(), "Process MAG data");

    DataFloat magraw_x,magraw_y,magraw_z;
    memcpy(&magraw_x.data, &frame.data[0], 4);
    memcpy(&magraw_y.data, &frame.data[4], 4);
    memcpy(&magraw_z.data, &frame.data[8], 4);
    mag_data_.magraw_x=magraw_x.f;
    mag_data_.magraw_y=magraw_y.f;
    mag_data_.magraw_z=magraw_z.f;
    mag_publisher();


}
    

void UavBase::processUwbData(DataFrame &frame)
{
    RCLCPP_INFO(this->get_logger(), "Process UWB data");
    DataFloat uwb_x,uwb_y,uwb_z;
    memcpy(&uwb_x.data,&frame.data[0],4);
    memcpy(&uwb_y.data,&frame.data[4],4);
    memcpy(&uwb_z.data,&frame.data[8],4);
    uwb_data_.pos_x=uwb_x.f;
    uwb_data_.pos_y=uwb_y.f;
    uwb_data_.pos_z=uwb_z.f;
    if(use_uwb_)
        uwb_publisher();
}
void UavBase::processMotorPWMData(DataFrame &frame)
{
    RCLCPP_INFO(this->get_logger(), "Process MOTOR PWM data");
    memcpy(&pwm_data_.pwm_0,&frame.data[0],2);
    memcpy(&pwm_data_.pwm_1,&frame.data[2],2);
    memcpy(&pwm_data_.pwm_2,&frame.data[4],2);
    memcpy(&pwm_data_.pwm_3,&frame.data[6],2);
    motorpwm_publisher();

}
void UavBase::processRcData(DataFrame &frame)
{
    RCLCPP_INFO(this->get_logger(), "Process RC data");
    memcpy(&rc_data_.ppm_0,&frame.data[0],2);
    memcpy(&rc_data_.ppm_1,&frame.data[2],2);
    memcpy(&rc_data_.ppm_2,&frame.data[4],2);
    memcpy(&rc_data_.ppm_3,&frame.data[6],2);
    memcpy(&rc_data_.ppm_4,&frame.data[8],2);
    memcpy(&rc_data_.ppm_5,&frame.data[10],2);
    memcpy(&rc_data_.ppm_6,&frame.data[12],2);
    memcpy(&rc_data_.ppm_7,&frame.data[14],2);
    rc_publisher();

}
void UavBase::processFlowData(DataFrame &frame)
{
    RCLCPP_INFO(this->get_logger(), "Process FLOW data");
    DataFloat pos_x_,pos_y_,speed_x,speed_y;
    memcpy(&pos_x_.data, &frame.data[0], 4);
    memcpy(&pos_y_.data, &frame.data[4], 4);
    memcpy(&speed_x.data, &frame.data[8], 4);
    memcpy(&speed_y.data, &frame.data[12], 4);
    flow_data_.pos_x=pos_x_.f;
    flow_data_.pos_y=pos_y_.f;
    flow_data_.speed_x=speed_x.f;
    flow_data_.speed_y=speed_y.f;
    if(use_flow_)
        flow_publisher();



}





bool UavBase::imu_calibration()
{
    DataFrame configFrame;

    // 封装IMU校准命令的数据帧
    configFrame.header = 0x55;
    configFrame.id     = 0x07;
    configFrame.length = 0x06;
    configFrame.data[0]= 0x00;
    configFrame.data[1]= 0x00;
    configFrame.data[2]= 0x00;
    configFrame.data[3]= 0x00;
    configFrame.data[4]= 0xFF;
    configFrame.data[5]= 0xFF;
    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + 
                         configFrame.data[3] + configFrame.data[4] + configFrame.data[5]) & 0xff;
    configFrame.tail   = 0xbb; 

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }

    return true;
}
//动捕数据尚待解决
void UavBase::mocap_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // ROS_INFO("I heard the pose from the robot"); 
    // ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    // ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);
    std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
    DataFloat mocap_pos_x,mocap_pos_y,mocap_pos_z,mocap_ori_x,mocap_ori_y,mocap_ori_z,mocap_ori_w;
    DataFrame mocapFrame;

    mocap_pos_x.f=msg->pose.position.x;
    mocap_pos_y.f=msg->pose.position.y;
    mocap_pos_z.f=msg->pose.position.z;
    mocap_ori_x.f=msg->pose.orientation.x;
    mocap_ori_y.f=msg->pose.orientation.y;
    mocap_ori_z.f=msg->pose.orientation.z;
    mocap_ori_w.f=msg->pose.orientation.w;
    memcpy(&mocapFrame.data[0],&mocap_pos_x.data,4);
    memcpy(&mocapFrame.data[4],&mocap_pos_y.data,4);
    memcpy(&mocapFrame.data[8],&mocap_pos_z.data,4);
    memcpy(&mocapFrame.data[12],&mocap_ori_x.data,4);
    memcpy(&mocapFrame.data[16],&mocap_ori_y.data,4);
    memcpy(&mocapFrame.data[20],&mocap_ori_z.data,4);
    memcpy(&mocapFrame.data[24],&mocap_ori_w.data,4);
    // 封装速度命令的
    mocapFrame.header = 0x55;
    mocapFrame.id     = 0x09;
    mocapFrame.length = 0x1c;
    mocapFrame.tail   = 0xbb;

    int len=mocapFrame.length;
    uint8_t sum=0;
    for(int i=0;i<len;i++)
    {   
        sum+=mocapFrame.data[i];

    }
    mocapFrame.check = sum & 0xff;

    try
    {
        serial_.write(&mocapFrame.header, sizeof(mocapFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }

}


void sigintHandler(int sig)
{
    sig = sig;
    
    printf("Uav shutdown...\n");

    serial::Serial serial;
    serial.setPort("/dev/ttyS3");                                   //选择要开启的串口号
    serial.setBaudrate(115200);                                     //设置波特率
    serial::Timeout timeOut = serial::Timeout::simpleTimeout(2000); //超时等待
    serial.setTimeout(timeOut);                                     
    serial.open();                                                  //开启串口

    // 如果串口打开，则驱动读取数据的线程
    if (serial.isOpen())
    {
        // 程序退出时自动停车
        DataFrame cmdFrame;
        cmdFrame.data[0] = 0x00;
        cmdFrame.data[1] = 0x00;         
        cmdFrame.data[2] = 0x00;
        cmdFrame.data[3] = 0x00;
        cmdFrame.data[4] = 0x00; 
        cmdFrame.data[5] = 0x00;
        cmdFrame.check = (cmdFrame.data[0] + cmdFrame.data[1] + cmdFrame.data[2] + 
                        cmdFrame.data[3] + cmdFrame.data[4] + cmdFrame.data[5]) & 0xff;

        // 封装速度命令的数据帧
        cmdFrame.header = 0x55;
        cmdFrame.id     = 0x01;
        cmdFrame.length = 0x06;
        cmdFrame.tail   = 0xbb;

        try
        {
            serial.write(&cmdFrame.header, sizeof(cmdFrame)); //向串口发数据
            printf("Execute auto stop\n");
        }
        catch (serial::IOException &e)
        {
            printf("Unable to send data through serial port\n"); //如果发送数据失败,打印错误信息
        }           
    }

    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    // 初始化ROS节点
    rclcpp::init(argc, argv);

    // 创建信号处理函数
    signal(SIGINT, sigintHandler);

    // 创建机器人底盘类，通过spin不断查询订阅话题
    rclcpp::spin(std::make_shared<UavBase>("uav_base"));
    
    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();

    return 0;
}
