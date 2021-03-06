//
//
// llab_01
// legs_follower
//
// Find and follow legs
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Legs_follower {
public:
    Legs_follower();
    void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
    void emergencyStop();
    void calculateCommand();
    void mainLoop();
    static const double EMERGENCY = 0.3;
    static const double EDGE_THRESHOLD = 1;
    static const double KNOWN_WIDTH = 8;
    static const double KNOWN_BETWEEN = 10;
    static const double KNOWN_DISTANCE = 1.5;
    static const double DEVIATION = 10;

protected:

    // Nodehandle for Random_mover robot
    ros::NodeHandle m_nodeHandle;

    // Subscriber and membervariables for our laserscanner
    ros::Subscriber m_laserSubscriber;
    sensor_msgs::LaserScan m_laserscan;

    // Publisher and membervariables for the driving commands
    ros::Publisher m_commandPublisher;
    geometry_msgs::Twist m_roombaCommand;

};

// constructor
Legs_follower::Legs_follower() {
    // Node will be instantiated in root-namespace
    ros::NodeHandle m_nodeHandle("/");

    // Initialising the node handle
    m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Legs_follower::laserCallback, this);
    m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Random_mover constructor



// callback for getting laser values
void Legs_follower::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

    if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
        m_laserscan.ranges.resize((size_t)scanData->ranges.size());
        }

    for(unsigned int i = 0; i < scanData->ranges.size(); i++)
    {
        m_laserscan.ranges[i] = scanData->ranges[i];
    }
}// end of laserCallback



// robot shall stop, in case anything is closer than ...
void Legs_follower::emergencyStop() {

        // see if we have laser data
     if( (&m_laserscan)->ranges.size() > 0)
    {
        for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
        {
            if( m_laserscan.ranges[i] <= EMERGENCY) {
                m_roombaCommand.linear.x = 0.0;
                m_roombaCommand.angular.z = 0.0;
                ROS_INFO("------------------ EMERGENCY! ------------------");
            }// end of if too close
        }// end of for all laser beams
    } // end of if we have laser data
}// end of emergencyStop



// here we go
// this is the place where we will generate the commands for the robot
void Legs_follower::calculateCommand() {
      // see if we have laser data
      if( (&m_laserscan)->ranges.size() > 0)
      {
        //get derivative
        double derivative[540];
        derivative[0] = 0;
        derivative[539] = 0;
        FILE* scan = fopen("r.dat", "w");
        for(unsigned int i=1; i<539; i++)
            derivative[i] = (1.0/2.0)*(-m_laserscan.ranges[i-1] + m_laserscan.ranges[i+1]);

        ROS_INFO("-----------Finding edges--------------");
        double edges[540];
        for(int i=0; i<540; i++)
            edges[i] = 0;
        int edgesCount = 0;
        for( int i=0; i<540; i++) {
            if(derivative[i]<=-0.8 || derivative[i]>=0.8) {
                edges[i] = derivative[i];
                edgesCount++;
            }
        }

        //no need in further calculations if there are no edges
        if(edgesCount == 0) {
            for(int i = 0; i< 540; i++)
              fprintf(scan, "%d %f %f %f\n",
                i,
                m_laserscan.ranges[i],
                derivative[i],
                edges[i]);
            fclose(scan);
              return;
        }

        ROS_INFO("-----------Reducing edges--------------");
        double reducedEdges[540];
        int reducedCount = 0, currentSame = -1;
        for(int i=0; i<540; i++){
            //we have edge on i
            if(edges[i]!=0 && i>0) {
                //have currently observed edge
                if(currentSame!=-1) {
                    //negative - getting closer, so need only last
                    //for legs it will be the point on leg
                    if(edges[i] < 0 && reducedEdges[currentSame] < 0) {
                        reducedEdges[i-1] = 0;
                        reducedEdges[i] = edges[i];
                        currentSame = i;
                    }
                    //positive - getting far, so need only first, that is on leg
                    if(edges[i] > 0 && reducedEdges[currentSame] > 0)
                        reducedEdges[i] = 0;
                    //sign changed - copy value and change currently observed edge
                    if(edges[i] > 0 && reducedEdges[currentSame] < 0 ||
                      edges[i] < 0 && reducedEdges[currentSame] > 0) {
                        reducedEdges[i] = edges[i];
                        currentSame = i;
                    }
                }
                //no currently observed edge - just copy and start observation
                else {
                    reducedEdges[i] = edges[i];
                    currentSame = i;
                }
            }
            //no edge on i - just copy 0 and no currently observed edge
            else {
                reducedEdges[i] = edges[i];
                currentSame = -1;
            }
        }
        //display edges
        char infoStringEdges[55];
        bool hasPositive, hasNegative;
        for(unsigned i=0; i<54; i++) {
            hasPositive = false;
            hasNegative = false;
            for(unsigned int j=i*10; j<(i+1)*10; j++) {
                if(reducedEdges[j]>0)
                    hasPositive = true;
                if(reducedEdges[j]<0)
                    hasNegative = true;
            }
            if(hasPositive && !hasNegative)
                infoStringEdges[i] = '^';
            if(hasNegative && !hasPositive)
                infoStringEdges[i] = 'v';
            if(hasPositive && hasNegative)
                infoStringEdges[i] = '#';
            if(!hasPositive && !hasNegative)
                infoStringEdges[i] = '_';
        }
        infoStringEdges[54] = 0;
        ROS_INFO("%s", infoStringEdges);

        //try find legs
        ROS_INFO("-----------Detecting possible legs--------------");
        int legs[540];
        int legStart = -1, widthCounter, legsCount = 0;
        double averageDistance = 0, probableWidth;
        for(int i=0; i<540; i++)
              legs[i] = 0.0;
        for(int i = 0; i< 540; i++) {
            if(reducedEdges[i]<0)
                legStart = i;
            if(legStart != -1 && reducedEdges[i]>0) {
                ROS_INFO("------------Have leg start at %d and leg end at %d", legStart, i);
                widthCounter = 0;
                averageDistance = 0;
                while(legStart + widthCounter <= i) {
                    averageDistance += m_laserscan.ranges[legStart + widthCounter];
                    widthCounter++;
                }
                ROS_INFO("------------Average distance %f", averageDistance/widthCounter);
                probableWidth = KNOWN_WIDTH * (averageDistance/widthCounter) / KNOWN_DISTANCE;
                ROS_INFO("------------Displayed width %d, wanted width %f", i - legStart, probableWidth);
                if(i - legStart <= probableWidth + DEVIATION && i - legStart >= probableWidth - DEVIATION) {
                    widthCounter = legStart;
                    while(widthCounter <= i) {
                        legs[widthCounter] = 1;
                        widthCounter++;
                    }
                    legsCount++;
                }
            }
        }
        //display legs
        char infoStringLegs[55];
        bool hasLeg;
        for(unsigned i=0; i<54; i++) {
            hasLeg = false;
            for(unsigned int j=i*10; j<(i+1)*10; j++)
                if(legs[j]==1)
                    hasLeg = true;
            if(hasLeg)
                infoStringLegs[i] = '#';
            else
                infoStringLegs[i] = '_';
        }
    infoStringLegs[54] = 0;
        ROS_INFO("%s", infoStringLegs);

        //no need in further calculations if there are no legs
          if(legsCount == 0) {
                for(int i = 0; i< 540; i++)
                      fprintf(scan, "%d %f %f %f %f %d \n",
                          i,
                          m_laserscan.ranges[i],
                          derivative[i],
                          edges[i],
                          reducedEdges[i],
                          legs[i]);
                fclose(scan);
                return;
          }

        //detecting men
        ROS_INFO("-----------Detecting possible men--------------");
        int men[540];
        int legFinishedAt = -1, legStartedAt = -1, betweenCounter;
        double probableBetween;
        for(int i=0; i<540; i++)
              men[i] = 0;
        for(int i = 0; i< 540; i++) {
            if(legs[i]==0 && legStartedAt!=-1) {
                legFinishedAt = i-1;
                legStartedAt = -1;
            }
            if(legs[i]==1 && legStartedAt==-1) {
                legStartedAt = i;
                if(legFinishedAt!=-1) {
                    ROS_INFO("------------Have first leg finished at %d and second leg started at %d", legFinishedAt, i);
                    averageDistance = (m_laserscan.ranges[legFinishedAt] + m_laserscan.ranges[legStartedAt])/2.0;
                    ROS_INFO("------------Average distance %f", averageDistance);
                    probableBetween = KNOWN_BETWEEN * averageDistance / KNOWN_DISTANCE;
                    ROS_INFO("------------Displayed between %d, wanted between %f", i - legFinishedAt, probableBetween);
                    if(i - legFinishedAt <= probableBetween + DEVIATION && i - legFinishedAt >= probableBetween - DEVIATION) {
                        betweenCounter = legFinishedAt;
                        while(betweenCounter <= i) {
                            men[betweenCounter] = 1;
                            betweenCounter++;
                        }
                    }
                    legFinishedAt = -1;
                }
            }
        }
        //display men
        char infoStringMen[55];
        bool hasMan;
        for(unsigned i=0; i<54; i++) {
            hasMan = false;
            for(unsigned int j=i*10; j<(i+1)*10; j++)
                if(men[j]==1)
                    hasMan = true;
            if(hasMan)
                infoStringMen[i] = '#';
            else
                infoStringMen[i] = '_';
        }
        infoStringMen[54] = 0;
        ROS_INFO("%s", infoStringMen);

        for(int i = 0; i< 540; i++)
                fprintf(scan, "%d %f %f %f %f %d %d \n",
                      i,
                      m_laserscan.ranges[i],
                      derivative[i],
                      edges[i],
                      reducedEdges[i],
                      legs[i],
                      men[i]);
        fclose(scan);

        //move to the man
        hasMan = false;
        for(int i = 0; i< 540; i++) {
            if(men[i]==1){
                m_roombaCommand.linear.x = (m_laserscan.ranges[i-2]-0.5)/20;
                m_roombaCommand.angular.z = (i-270)/200;
                hasMan = true;
                break;
            }
        }
        if(!hasMan) {
            m_roombaCommand.linear.x = 0;
            m_roombaCommand.angular.z = 0;
        }
    }//end of have laser

} // end of calculateCommands


//
void Legs_follower::mainLoop() {
    // determines the number of loops per second
    ros::Rate loop_rate(20);

    // als long as all is O.K : run
    // terminate if the node get a kill signal
    while (m_nodeHandle.ok())
    {
        calculateCommand();
        emergencyStop();

        // send the command to the roomrider for execution
        m_commandPublisher.publish(m_roombaCommand);

        // spinOnce, just make the loop happen once
        ros::spinOnce();
        // and sleep, to be aligned with the 50ms loop rate
        loop_rate.sleep();
    }
}// end of mainLoop


int main(int argc, char** argv) {

    // initialize
    ros::init(argc, argv, "Leg_follower");

    // get an object of type Legs_follower and call it robby
    Legs_follower robby;

    // main loop
    robby.mainLoop();

    return 0;

}// end of main

// end of file: legs_follower.cpp
