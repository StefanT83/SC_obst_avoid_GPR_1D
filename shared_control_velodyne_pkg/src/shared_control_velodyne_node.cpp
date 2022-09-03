/* Obstacle avoidance shared control algo for usage on teleoperated robots, using as inputs:
- lidar: Velodyne: one ring of data
- joypad: Sony DS4: right joystick data
- ?: linear velocity

 Sys coordinates conventions:
 - frame rviz [moving frame attached to sensor]: how data is stored in topic /velodyne_points, and wrt which we visualize data in rviz (the red-green-blue axes)
 - frame vld [moving frame attached to sensor]: obtained by rotating rviz by 90 deg, see drawing below
           xUrvizLobst == yUvldLobst   ^    an obstacle situated to the right of the sensor
                                     |   /
                                     | /
                  yUrvizLobst <------+-------> xUvldLobst
- frame b [moving frame attached to sensor]: obtained by rotating rviz by summing 90 deg (thus obtaining frame vld) + phiUbLmisalign, where phiUbLmisalign is the angle betwenn xUrviz and yUb, expressed wrt/in-coord-of frame b

 Code (architecture) inspired from:
  https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/src/
  https://github.com/StefanT83/ASC_policy_iter/blob/main/ROS/src/shared_control_policy_iter_PMspeed5_node.cpp
  https://wiki.ros.org/pcl/Overview

Code resources:
 https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
 https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
*/

#include <ros/ros.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include <vector>

// a point on an obstacle identified by velodyne
struct PointObst {
   double xUbLobst, yUbLobst, distUbLobst, angleUbLobst, intensity;
}; //struct PointObst

class SharedControl_velodyne {
private:
   // // Part 1 - velodyne config: notations largely inspired by https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/src/velodyne_laserscan.cpp
   static constexpr float RESOLUTION {0.007}; //[rad] angle increments for storing ring data; value cf https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/cfg/VelodyneLaserScan.cfg , also verified with a running VLP-16 by adding code ROS_INFO("%f",RESOLUTION) inside velodyne_laserscan.cpp
   static const size_t SIZE = 2.0*M_PI/RESOLUTION; //[-] narrowing conversion (loss of data); number of bins i.e. data points corresp to one data ring; size_t is "long unsigned int" namely "%lu" specifier in printf() and ROS_INFO()

   // Choose one ring among the 16 rings of VLP-16: this corresp to a 2D plane cut
   static constexpr uint16_t ring {8}; // VLP-16, same notation cf https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/src/velodyne_laserscan.cpp

   // Def structure of PointCloud2 packet: values according to analysing $rostopic echo /velodyne_points
   //  Explanations: point_step=22 [bytes] allocated to the content of one 3D point = 4 bytes (FLOAT32 i.e. datatype=7) for "x" + 4 bytes (FLOAT32) for "y" + 4 bytes (FLOAT32) for "z" + 4 bytes (FLOAT32) for "intensity" + 2 bytes (UINT16 i.e. datatype=4) for "ring" + 4 bytes (FLOAT32) for "time"
   //  Visual repres:
   //  x         y         z          intensity   ring    time      <- fields
   //  _ _ _ _   _ _ _ _   _ _ _ _    _ _ _ _     _ _     _ _ _ _   <- represent each one byte
   //  0         4         8         12          16      18         <- define the "offset"
   //
   // Other data members: count=1 means 1 entity/element in the succession of bytes between the "offsets"; row_step=638792 = (point_step=22[bytes]) x 29036 [entities/elements/3D-points], the row_step seems to vary from one measurement to another
   static constexpr int offset_x         {0}; //[-]
   static constexpr int offset_y         {4}; //[-]
   static constexpr int offset_z         {8}; //[-]
   static constexpr int offset_intensity {12}; //[-]
   static constexpr int offset_ring      {16}; //[-]

   // // Part 2: physical qtts
          const double SD2_absvmax_fwd  {0.4}; //[m/s]
          const double SD2_absvmax_bwd  {-SD2_absvmax_fwd}; //[m/s] to-do: check experimentally the value
          const double SD2_chassisWidth {SharedControl_velodyne::inch2m(20.0)}; //[m] chassisWidth = 20 inch cf 'PDF Drawing' on https://www.superdroidrobots.com/robots/prebuilt-robots/sold-custom-robots/product=2815
          const double SD2_omegamax     {2.0*SD2_absvmax_fwd/SD2_chassisWidth}; // [rad/s] used formula angle*radius=length, with radius = SD2_chassisWidth/2, then differentiate; see also whc_parameters1.m
          const double distSensorToEdgeOfRobot = .15; //[m] to-do: check experimentally the value; dist from velodyne sensor to the fwd edge of the chassis width

   // choose/define accordingly
   static constexpr double sigma_v_nz     {.01}; //[-]
   static constexpr double sigma_omega_nz {.03}; //[-]

          const double phiUbLmisalign = deg2rad(0.0); //[rad] see def above, this angle captures the fact that the velodyne sensor axis pointing fwd (namely xUrviz == yUvld) is not perfectly parallel to the lenght of the SD2 robot

   // // Part 3 - data members
   double vd_nz, omegad_nz, v_nz; //normalized values
   double v, omega; //odometry: v [m/s]; omega [rad/s]

   std::vector<PointObst> scan;

   ros::NodeHandle nh_; // in C++ the naming convention of a variable with an underscore usually indicates a private member variable: see B2017Newman, Comment p55
   ros::Subscriber velodyne_sub_, joypad_sub_, odom_sub_;
   ros::Publisher  joypad_SC_pub_;

   // member function
   void velodyneSubscCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
     ROS_INFO_ONCE("VelodyneLaserScan: Extracting ring %u", ring);

     const size_t id_xUrvizLobst = 0; //=offset_x/4
     const size_t id_yUrvizLobst = 1; //=offset_y/4
     const size_t id_intensity = offset_intensity/4;
     const size_t id_ring = offset_ring/4;

     for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it)	{
       const uint16_t ring_act = *((const uint16_t*)(&it[id_ring]));  // actual ring

       //V&V
       //std::cout << "ring_act=" << ring_act << '\n';
       //sleep(.01);

       if (ring_act == ring) {
         const float xUrvizLobst = it[id_xUrvizLobst];  // Axes convention (viewed from above): xUrvizLobst is fwd (red in rviz), yUrvizLobst is towards left (green in rviz), zUbLobst is upwards (green in rviz); this coord sys is different than the one in the manual [VLP-16-User-Manual.pdf, \S 9, p54]
         const float yUrvizLobst = it[id_yUrvizLobst];     // see axes convention above

         // conseq
         const int bin = (atan2(yUrvizLobst, xUrvizLobst) + static_cast<double>(M_PI))/RESOLUTION; // bin=0 corresp to the 3D point located straight behind the sensor, i.e. along xUrvizLobst-axis but looking towards the negative values

         //V&V
         //ROS_INFO("%f -- %f -- %d",atan2(yUrvizLobst,xUrvizLobst) + static_cast<float>(M_PI),RESOLUTION,bin);

         if ((bin >= 0) && (bin < static_cast<int>(SIZE))) {
           transform_rviz2b(scan[bin].xUbLobst, scan[bin].yUbLobst, xUrvizLobst, yUrvizLobst);

           scan[bin].distUbLobst  = sqrtf(pow(scan[bin].xUbLobst,2.0) + pow(scan[bin].yUbLobst,2.0));
           scan[bin].angleUbLobst = atan2(scan[bin].yUbLobst, scan[bin].xUbLobst); //[rad] angleUbLobst=0 corresp to obstacle being on xUbLobst-axis; angle defined wrt coord sys (oUbLobst,xUbLobst,yUbLobst,zUbLobst) described above
           scan[bin].intensity    = it[id_intensity];
         } // if(.)

       } // if (ring_act == ring)
     } // for (.)

     /*
     //V&V
     ROS_INFO("Min dist to obstacle on this ring is %f",*std::min_element(scan_distUbLobst.begin(), scan_distUbLobst.end()));
     ROS_INFO("Max dist to obstacle on this ring is %f",*std::max_element(scan_distUbLobst.begin(), scan_distUbLobst.end()));

     ROS_INFO("Min intensity on this ring is %f",*std::min_element(scan_intensity.begin(), scan_intensity.end()));
     ROS_INFO("Max intensity on this ring is %f",*std::max_element(scan_intensity.begin(), scan_intensity.end()));

     ROS_INFO("Min angle on this ring is %f",*std::min_element(scan_angle.begin(), scan_angle.end()));
     ROS_INFO("Max angle on this ring is %f",*std::max_element(scan_angle.begin(), scan_angle.end()));

     for (float x:scan_angle) printf(" %0.2f",x); //std::cout << x << " ";
     */

     // testing-purpose only
     //scan[1].xUbLobst = nan("");

     // handle any nan present in the data (recall nan was used to initialize scan[all].X): wcs approach by artificially adding harmless data;
     for (auto bin=0; bin<scan.size(); bin++) {
       if ( isnan(scan[bin].xUbLobst) || isnan(scan[bin].yUbLobst) || isnan(scan[bin].angleUbLobst) || isnan(scan[bin].distUbLobst) || isnan(scan[bin].intensity)) {
         ROS_INFO("Found nan within scan, so action will be taken:");

         // associate wcs values
         scan[bin].distUbLobst  = 33.0; //(close-to) max range of velodyne sensor
         scan[bin].intensity    = 1.0;  //lowest intensity value


         // conseq: extract (xUbLobst,yUbLobst) from (distUbLobst,angle)
         double xUrvizLobst_temp, yUrvizLobst_temp;
         inv_atan2(xUrvizLobst_temp, yUrvizLobst_temp, static_cast<double>(bin)*static_cast<double>(RESOLUTION) - static_cast<double>(M_PI), scan[bin].distUbLobst);
         transform_rviz2b(scan[bin].xUbLobst, scan[bin].yUbLobst, xUrvizLobst_temp, yUrvizLobst_temp);

         // conseq: extract angle
         scan[bin].angleUbLobst = atan2(scan[bin].yUbLobst,scan[bin].xUbLobst); // alternative method: the index-value i.e. bin value embeds info about the angle

         ROS_INFO(" The point (obstacle) values were artificially adjusted to: angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(scan[bin].angleUbLobst), scan[bin].distUbLobst, scan[bin].xUbLobst, scan[bin].yUbLobst, scan[bin].intensity);
       } // if
     } //for (.)

     //V&V: visalise scan
     ROS_INFO("scan.size()=%lu", scan.size());
     for (unsigned long bin=0; bin<scan.size(); bin+=10) {
       ROS_INFO("scan[%lu].distUbLobst=%f",bin,scan[bin].distUbLobst);
     } //for (auto bin=0; ..)

     // call: as soon as we have new info about obstacles
     sharedControlAlgo();

   } //void velodyneSubscCallback

   // member function
   void joypadSubscCallback(const sensor_msgs::JoyConstPtr& msg) {
     // using axis convention (v_joy,omega_joy) from article [TeZhCa2020] i.e. art_whc2
      double v_joy = msg->axes[4]; //[-] \in [-1,1] the 5th element inside array axes represents advancing bwd/fwd user intention (max fwd = +1; max bwd = -1)
      double omega_joy = msg->axes[3]; // \in [-1,1]  the 4th element inside array axes represents turning right/left user intention (max left = +1; max right=-1)

      // conclude
      vd_nz     = v_joy; //[-]
      omegad_nz = omega_joy; //[-]

      // V&V
      ROS_INFO("joypad data: %f -- %f",vd_nz,omegad_nz);
   } //void velodyneSubscCallback(.)

   // member function
   void odometrySubscCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    v = msg->twist.twist.linear.x;  //[m/s]
    omega = msg->twist.twist.angular.z; //[rad/s]
    //ROS_INFO("Raw     : Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V

    // postprocess: saturate: safety reasons (data consistency)
    v = std::min(SD2_absvmax_fwd, std::max(SD2_absvmax_bwd, v)); //overwrite
    omega = std::min(SD2_omegamax, std::max(-SD2_omegamax, omega)); //overwrite
    //ROS_INFO("Postproc: Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V
   } //void odometrySubscCallback(.)

   // member function
   void sharedControlAlgo() {
     // Output: (vr_nz,omega_nz)
     ROS_INFO("Entered sharedControlAlgo()");

     // ini: by default assume no need to enable ASC
     double vr_nz {vd_nz};
     double omegar_nz {omegad_nz};

     // V&V
     //publish_joy_SC(vr_nz,omegar_nz);

     // // // Main Algo
     // normalize values
     double v_nz     = v>0 ? v/SD2_absvmax_fwd : v/SD2_absvmax_bwd;
     double omega_nz = omega/SD2_omegamax;

///*
     // // case1. vehicle advancing straight ahead (= fwd + wo rotation)
     ROS_INFO("Case1: check conditions: v_nz=%.3f; omega_nz=%.3f; vd_nz=%.3f",v_nz,omega_nz,vd_nz);
     if ((v_nz>3*sigma_v_nz) && (abs(omega_nz)<=3*sigma_omega_nz)) //vehicle advancing fwd + wo rotation
      if (vd_nz>0) { //user expresses intention to advance fwd;
        //then it is meaningful/makes-sense to apply ASC
        ROS_INFO("Case1: entered");

        // // step1. find closest obstacle
        // ini
        PointObst closestPointObstStraightAhead {0, 33, 33, M_PI, 1};

        // check all points
        for (auto id=1; id<scan.size(); id++ ) { // index starts at 1 because the ini point was set to scan[0]
          if (scan[id].distUbLobst<closestPointObstStraightAhead.distUbLobst) { //then we have found a candidate for closestPointObst
            //ROS_INFO("Case1: Found closer point (obstacle): angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(closestPointObstStraightAhead.angleUbLobst), closestPointObstStraightAhead.distUbLobst, closestPointObstStraightAhead.xUbLobst, closestPointObstStraightAhead.yUbLobst, closestPointObstStraightAhead.intensity);

            //check whether the candidate for closestPointObst is straight ahead
            if (abs(scan[id].xUbLobst) <= SD2_chassisWidth/2.0)
              closestPointObstStraightAhead = scan[id];
          } // if
        } //for (.)

        ROS_INFO("Case1: The closest point (obstacle) is: angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(closestPointObstStraightAhead.angleUbLobst), closestPointObstStraightAhead.distUbLobst, closestPointObstStraightAhead.xUbLobst, closestPointObstStraightAhead.yUbLobst, closestPointObstStraightAhead.intensity);

        // // step2. calc (vr_nz,omega_nz)
        vr_nz     = std::min(calc_vrMax_nz(closestPointObstStraightAhead.distUbLobst-distSensorToEdgeOfRobot), vd_nz);
        omegar_nz = omegad_nz * vr_nz/vd_nz;

        ROS_INFO("Case1: User desires (vd_nz=%.3f,omegad_nz=%.3f), and SC issues (vr_nz=%.3f,omegar_nz=%.3f)", vd_nz,omegad_nz, vr_nz,omegar_nz);
      } //if (vd_nz>0)

      // // case2. vehicle advancing fwd + w rotation
      // WIP..

      // // conclude
      publish_joy_SC(vr_nz,omegar_nz);

//*/


/*
     // // Algo: find closest obstacle and show its angle
     // ini
     PointObst closestPointObst = scan[0];

     // check all points
     for (auto id=1; id<scan.size(); id++ ) { // index starts at 1 because the ini point was set to scan[0]
       if (scan[id].distUbLobst<closestPointObst.distUbLobst) {
         closestPointObst = scan[id];
       } // if
     } //for (.)

     // print result
     ROS_INFO("The closest point (obstacle) is: angleUbLobst=%f [deg], distUbLobst=%f [m], xUbLobst=%f [m], yUbLobst=%f [m], intensity=%f [?]",rad2deg(closestPointObst.angleUbLobst), closestPointObst.distUbLobst, closestPointObst.xUbLobst, closestPointObst.yUbLobst, closestPointObst.intensity);
*/

   } //void sharedControlAlgo()

   // member function
   double calc_vrMax_nz(double xUobst) {  // formula/relation:  dUbLobst = distSensorToEdgeOfRobot + xUobst
     // Models the relation between vrUmax_nz and dUbLobst, as a function
     // Output: vrUmax_nz [-]

      // //[var1] linear fct fct(x)=a*x+b, intersecting the points (x=d0,vrMax=0) and (x=dmax,vrMax=1)
     // Choose
     double d0   = 0.1; //[m] d0 is where fct=fct(xUobst) starts ramping up
     double dmax = 0.62; //[m] dmax is the distance-to-obstacle where the SC kicks in

     // ini: safe value
     double vrMax_nz = nan("");

     if (xUobst<0.0) {
       ROS_ERROR("Please check xUobst");
     } else if (xUobst<=d0) {
       vrMax_nz = 0.0;
     } else if (xUobst<=dmax) {
       vrMax_nz = (xUobst-d0)/(dmax-d0);
     } else {
       vrMax_nz = 1.0;
     } //if (.)

     //conclude
     return vrMax_nz;
   } // double calc_vrMax_nz(double xUobst)

   // member function: helper/utility function
   void publish_joy_SC(double vr_nz, double omegar_nz) {
      sensor_msgs::Joy joy_SC_msg;

      //ROS
      joy_SC_msg.header.stamp = ros::Time::now();

      joy_SC_msg.axes.push_back(0.0); //axes[0]
      joy_SC_msg.axes.push_back(0.0); //axes[1]
      joy_SC_msg.axes.push_back(0.0); //axes[2]
      joy_SC_msg.axes.push_back(omegar_nz); //axes[3] ru
      joy_SC_msg.axes.push_back(vr_nz); //axes[4]
      joy_SC_msg.axes.push_back(0.0); //axes[5]
      joy_SC_msg.axes.push_back(0.0); //axes[6]
      joy_SC_msg.axes.push_back(0.0); //axes[7]

      for (auto i=0; i<13; i++) joy_SC_msg.buttons.push_back(0);

      joypad_SC_pub_.publish(joy_SC_msg);
   } //void publish_joy_SC()

   // member function
   void transform_rviz2b(double& xUbLobst, double& yUbLobst, const double& xUrvizLobst, const double& yUrvizLobst) {
   // Inputs: (xUrvizLobst,yUrvizLobst);   Outputs: (xUbLobst,yUbLobst)
   // pUb = RUbLrviz*pUrviz, where RUbLrviz:=R_{z,90[deg]+phiUbLmisalign} using notations [SpHuVi2005,p57]; pUb=def=[xUb; yUb]; pUrviz=def=[xUrviz; yUrviz];
   double phi_temp = M_PI/2 + phiUbLmisalign;
   xUbLobst = cos(phi_temp)*xUrvizLobst - sin(phi_temp)*yUrvizLobst;
   yUbLobst = sin(phi_temp)*xUrvizLobst + cos(phi_temp)*yUrvizLobst;
   } //void transform_rviz2b(.)

   // member function
   static void transform_b2rviz(double& xUrvizLobst, double& yUrvizLobst, const double& xUbLobst, const double& yUbLobst) {
   // Inouts: (xUrvizLobst,yUrvizLobst);   Outputs: (xUbLobst,yUbLobst)
   // pUb = RUbLrviz*pUrviz, where RUbLrviz:=R_{z,phi=90 deg}=def=[0 -1; 1 0]  using notations [SpHuVi2020]
   yUrvizLobst = -xUbLobst;
   xUrvizLobst = yUbLobst;
   } //void transform_rviz2b(.)

   // member function
   static inline void inv_atan2(double& x, double& y, const double& theta, const double& len) {
     // Given (theta,len) previously computed using atan2(y,x), now extract (x,y)
     // Inputs (angle,len);   Outputs: (x,y)
     x = len*cos(theta);
     y = len*sin(theta);
   } //

public:
   // constructor
   SharedControl_velodyne(ros::NodeHandle& nh) :nh_(nh), vd_nz(0.0), omegad_nz(0.0), v_nz(0.0) {
      // create an 'empty' PointObst
      PointObst temp {nan(""),nan(""),nan(""),nan(""),nan("")};

      // assign the correct size to vectors
      scan.resize(static_cast<unsigned long int>(SIZE),temp);

      //V&V
      //for (float x:scan_distUbLobst) std::cout << x << " ";

      velodyne_sub_  = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &SharedControl_velodyne::velodyneSubscCallback, this);
      joypad_sub_    = nh.subscribe<sensor_msgs::Joy>("/joy_raw", 1, &SharedControl_velodyne::joypadSubscCallback, this);
      odom_sub_      = nh.subscribe<nav_msgs::Odometry>("/wheel_odom", 1, &SharedControl_velodyne::odometrySubscCallback, this);
      joypad_SC_pub_ = nh_.advertise<sensor_msgs::Joy>("/joy_SC",1);

   } // SharedControl_velodyne(.)

   // destructor
   ~SharedControl_velodyne() {
	    ROS_INFO(" Ended Shared Control!\n");
   } // ~SharedControl_veldyne()

   // member function: helper/utility function
   constexpr static inline double rad2deg(double rad) {
     return rad*180.0/M_PI;
   } //double rad2deg(double rad)

   // member function: helper/utility function
   constexpr static inline double deg2rad(double deg) {
     return deg*M_PI/180.0;
   } //double deg2rad(double deg)

     // member function: helper/utility function
     constexpr static inline double inch2m(double inch) {
       return 0.0254*inch;
     } //double inch2m(double inch)

}; // class SharedControl_velodyne


// ++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv) {
    //ROS_INFO("Test: %f; %f; %lu",M_PI,SharedControl_velodyne::RESOLUTION, SharedControl_velodyne::SIZE);

    ros::init(argc, argv, "shared_control_velodyne_node");
    ros::NodeHandle nh;
    SharedControl_velodyne sc(nh);
    ros::spin();

    return 0;
} // int main(.)
