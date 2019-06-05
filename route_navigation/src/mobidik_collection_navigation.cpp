#include "mobidik_collection_navigation.h"
#include <ed_gui_server/EntityInfos.h>
#include <ropod_ros_msgs/DockingCommand.h>

#include "../../../../ED/ed/include/ed/termcolor.hpp"

/*--------------------------------------------------------*/
MobidikCollection::MobidikCollection( )
{
    true_bool_msg_.data = true;
};

/*--------------------------------------------------------*/
MobidikCollection::~MobidikCollection()
{
    base_position.reset();
};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
// Strongly inspired by https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
template<typename T>
// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment( T& p, T& q, T& r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}
 
 template<typename T>
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation( T& p, T& q, T& r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

 template<typename T>
// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect( T& p1, T& q1, T& p2, T& q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}

 template<typename T>
// Returns true if the point p lies inside the polygon[] with n vertices
bool isInside(std::vector<T> Points, T& p)
{
     int n = Points.size();   
        
    // There must be at least 3 vertices in polygon[]
    if (n < 3)  return false;
    
    // Create a point for line segment from p to infinite
    T extreme;
    extreme.x = INF;
    extreme.y = p.y;
 
    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
 
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(Points[i], Points[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(Points[i], p, Points[next]) == 0)
               return onSegment(Points[i], p, Points[next]);
            count++;
        }
        i = next;
    } while (i != 0);
    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}



 

bool MobidikCollection::getMobidik(const ed::WorldModel& world, ed::UUID* mobidikID) 

// add id of entity which is assumed to be the mobidik

// TODO via ED WM (if data-association solved!?)
{
/*        for (unsigned int ii = 0; ii < markerArray.markers.size(); ii++)
        {
                visualization_msgs::Marker markerToCheck = markerArray.markers.at(ii);
                if (markerToCheck.ns == "Mobidik")
                {
                        *marker = markerToCheck;
                        return true;
                }                    
        }   */   
        
        std::cout << "Get mobidik" << std::endl;
    for ( ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it )
    {
    

std::string laserID = "-laserTracking";
    const ed::EntityConstPtr& e = *e_it;

        if ( e->id().str().length() < laserID.length() )
        {
                continue;
        }
        if ( e->id().str().substr ( e->id().str().length() - laserID.size() ) != laserID ) 
        {
                continue;
        }

        std::cout << "entityID = " << e->id() << std::endl;
       ed::tracking::FeatureProperties property = e->property ( featureProperties );
//     for ( int ii = 0; ii < measuredProperties.size(); ii++ )
//     {
//         ed::tracking::FeatureProperties property = measuredProperties[ii];
//         bool possiblyMobidik = false;
        std::cout << "get mobidik: depth and width of object: = " << property.rectangle_.get_d() << ", " << property.rectangle_.get_w() << std::endl;
        if ( property.getFeatureProbabilities().get_pRectangle() > property.getFeatureProbabilities().get_pCircle() && // Dimension check
                property.rectangle_.get_d() < MOBIDIK_WIDTH + MOBIDIK_MARGIN &&
                property.rectangle_.get_w() < MOBIDIK_WIDTH + MOBIDIK_MARGIN &&
                ( property.rectangle_.get_d() > MOBIDIK_WIDTH - MOBIDIK_MARGIN ||
                  property.rectangle_.get_w() > MOBIDIK_WIDTH - MOBIDIK_MARGIN ) )
        {
std::cout << "Dimension check succesful for ent = " << e->id() << std::endl; // TODO what if multiple objects are found which fit in the specified dimensions inside the specified area?
ed::UUID mobidikId = e->id();

            for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
            {
                const ed::EntityConstPtr& e = *it;

                std::string MobiDikWaitingAreaID = "MobidikArea";

                if ( e->id().str().length() < MobiDikWaitingAreaID.length() )
                {
                    continue;
                }

                if ( e->id().str().substr ( 0, MobiDikWaitingAreaID.length() ) == MobiDikWaitingAreaID )
                {
                        // It is assumed here that there is a navigation task, so only points on the ground are taken into consideration
                        
                        std::vector<geo::Vector3> points = e.get()->shape().get()->getMesh().getPoints();
                        std::vector<geo::Vector3> groundPoints;
                        const geo::Vec3T<double> pose = e.get()->pose().getOrigin();
                        
                        for(unsigned int iPoints = 0; iPoints < points.size(); iPoints++)
                        {
                                if(points[iPoints].getZ() == 0)
                                {
                                        groundPoints.push_back( points[iPoints] + pose );
                                }
                        }        
                       geo::Vector3 mobidikPoint( property.getRectangle().get_x(), property.getRectangle().get_y(), property.getRectangle().get_z() );
                       
                    if( isInside( groundPoints, mobidikPoint) )
                    {
//                         possiblyMobidik = true; 
                            // assumption: there is only 1 mobidik in the specifik area
//                            mobidikEntity = *e_it;
                           std::cout << "*e_it = " << *e_it << std::endl;
                           std::cout << "Mobidik found for entity = " << e->id() << std::endl;
                              //  *mobidikID = *e_it;
                                *mobidikID = (*e_it)->id();
                           return true;
                    }
                }   
            }
        }        
    }

    return false;
};

template <class T>
void MobidikCollection::wrap2pi ( T *angle )
{
    *angle -= M_PI * std::floor ( *angle * ( 1 / ( M_PI )) ); // TODO does not always work
}

template <class T>
void MobidikCollection::wrap2twopi ( T *angle )
{
    *angle -= 2*M_PI * std::floor ( *angle * ( 1 / ( 2*M_PI )) ); // TODO does not always work
}

bool MobidikCollection::setMobidikPosition ( const ed::WorldModel& world,ed::UpdateRequest& req, std::string mobidikAreaID,   ed::UUID mobidikID, visualization_msgs::Marker* points ) 
// Store mobidik in ED as it might not be detected anymore when the robot rotates
{
       
    // Area where the mobidik will be collected is assumed to be known
        
        
//         Here, get mobidik ID and find entity of the mobidik
  /*
        geo::Pose3D mobidikPose;

    visualization_msgs::Marker mobidikPoseMarker = mobidikMarker;
   
    
    tf::Quaternion q ( mobidikPoseMarker.pose.orientation.x, mobidikPoseMarker.pose.orientation.y, mobidikPoseMarker.pose.orientation.z, mobidikPoseMarker.pose.orientation.w );
    tf::Matrix3x3 m ( q );
    double MD_roll, MD_pitch, MD_yaw;
    m.getRPY ( MD_roll, MD_pitch, MD_yaw );
    wrap ( &MD_yaw );

    double xPos = mobidikPoseMarker.pose.position.x;
    double yPos = mobidikPoseMarker.pose.position.y;

    geo::Vec3d origin ( xPos, yPos, mobidikPoseMarker.pose.position.z );
    mobidikPose.setOrigin ( origin );
*/
  
  ed::EntityConstPtr mobidikEntity;
  if(!getEntityPointer(world, mobidikID, mobidikEntity))
  {
          ROS_WARN("mobidik entity not found");
          return false;
  }
  
  std::cout << "setMobidikPosition"<< std::endl;
//    geo::Pose3D mobidikPose;
   double mobidikWidth, mobidikLength, MD_yaw, lastUpdateTimestamp;    
ed::tracking::FeatureProperties mobidikFeatures;    
//     for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
//     {
//       const ed::EntityConstPtr& e = mobidikID; 
//std::cout << "mobidikID = " << mobidikId << std::endl;
//         if( e->id() == mobidikId )
//         {
std::cout << "mobidikIDPtr = " << mobidikEntity<< std::endl;
std::cout <<"MobidikID = " << mobidikEntity->id() << std::endl;
		mobidikFeatures = mobidikEntity->property ( featureProperties );
                ed::tracking::Rectangle mobidikModel = mobidikFeatures.getRectangle();
std::cout << "mobidikFeatures are: " << std::endl;
// mobidikFeatures.printProperties();

                std::cout <<"xPos = " << mobidikModel.get_x()  << std::endl; 
                geo::Vec3d origin ( mobidikModel.get_x(), mobidikModel.get_y(), mobidikModel.get_z() );
//                 mobidikPose.setOrigin( origin );
                
                mobidikWidth = mobidikModel.get_w();
                mobidikLength = mobidikModel.get_d();

                MD_yaw = mobidikModel.get_yaw();
                wrap2twopi ( &MD_yaw );
                
                std::cout << "Mobidik yaw = " << MD_yaw << std::endl;
                
                lastUpdateTimestamp = mobidikEntity->lastUpdateTimestamp();
                
//                 continue;
//         }
//       
//     }
//    std::cout << "After mobidik found: " << std::endl;
// mobidikFeatures.printProperties();

        std::string orientationWPID = "orient_wp_" +  mobidikAreaID;
        orientationWPID_ = orientationWPID;
std::cout << "orientationWPID = " << orientationWPID << std::endl;
  ed::EntityConstPtr wpEntity;
  if(!getEntityPointer(world, orientationWPID, wpEntity))
  {
          ROS_WARN("orient_wp_ not found");
          return false;
  }

//     for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
//     {
//         const ed::EntityConstPtr& e = *it;


//         if ( orientationWPID.compare ( e.get()->id().str() ) == 0 ) // function returns 0 if strings are equal
//         {
            geo::Pose3D poseWP = wpEntity.get()->pose();
            geo::Quaternion rotationWP = poseWP.getQuaternion();

            tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
            tf::Matrix3x3 matrix ( q );
            double WP_roll, WP_pitch, WP_yaw;
            matrix.getRPY ( WP_roll, WP_pitch, WP_yaw );
            std::cout << "WP_yaw before = " << WP_yaw << std::endl;
            wrap2twopi ( &WP_yaw );
            
            std::cout << "WP_yaw = " << WP_yaw << std::endl;

            // find quadrant closest to the orientation of the mobidik
            geo::real diffAngleMin = INFINITY;
            double angle, angleAtMinDiff, deltaAngleCorrected;
            for ( unsigned int ii = 0; ii < 4; ii++ )
            {
                double deltaAngle = ii*M_PI_2;
                angle = MD_yaw + deltaAngle;
                wrap2twopi ( &angle );

                geo::real diffAngle = std::min ( std::fabs ( angle - WP_yaw ), std::fabs ( angle + 2*M_PI - WP_yaw ) );
                diffAngle = std::min ( diffAngle, std::fabs ( angle - 2*M_PI - WP_yaw ) );
                
                if ( diffAngle < diffAngleMin )
                {
                    diffAngleMin = diffAngle;
                    angleAtMinDiff = angle;
                    deltaAngleCorrected = deltaAngle;
                }
            }
            
            MD_yaw = angleAtMinDiff;
            if ( deltaAngleCorrected == M_PI_2 || deltaAngleCorrected == 3*M_PI_2 ) // change of pi/2 of 3*pi/2-> width and length change
            {
                double mobidikWidthOld = mobidikWidth;
                mobidikWidth = mobidikLength;
                mobidikLength = mobidikWidthOld;
            }

            geo::Mat3 rotation;
            rotation.setRPY ( 0.0, 0.0, MD_yaw );
//             mobidikPose.setBasis ( rotation );

            ed::ConvexHull chull;
            
            float length = std::sqrt ( std::pow (0.5* mobidikWidth, 2.0 ) + std::pow ( 0.5*mobidikLength, 2.0 ) );
// std::cout << "mobidik pose = " << mobidikPose.getOrigin().getX() << mobidikPose.getOrigin().getY() << std::endl;
            geometry_msgs::Point p;
            angle = MD_yaw + atan2(mobidikLength, mobidikWidth);
            geo::Vec2f point ( mobidikModel.get_x() + length*cos ( angle ), mobidikModel.get_y() + length*sin ( angle )  );
            chull.points.push_back ( point ); 
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
          
            angle = MD_yaw + atan2(mobidikLength, -mobidikWidth);
            point.x = mobidikModel.get_x() + length*cos ( angle ); point.y =  mobidikModel.get_y() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
            
            angle = MD_yaw + atan2(-mobidikLength, -mobidikWidth);
            point.x = mobidikModel.get_x() + length*cos ( angle ); point.y =  mobidikModel.get_y() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p ); 
            
            angle = MD_yaw + atan2(-mobidikLength, mobidikWidth);
            point.x = mobidikModel.get_x() + length*cos ( angle ); point.y =  mobidikModel.get_y() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
            
        //    ed::tracking::FeatureProperties entityProperties = mobidikFeatures; // TODO several properties double defined (both in key-properties and entitydescriptions) -> Make it consistent!
            mobidikFeatures.rectangle_.set_w(mobidikWidth);
            mobidikFeatures.rectangle_.set_d(mobidikLength);
            std::cout << "Before setting new properties." << std::endl;

            mobidikFeatures.rectangle_.set_yaw(MD_yaw);
//             mobidikFeatures.printProperties();

std::cout << "MD_yaw set at " << MD_yaw << std::endl;
            
            // Mark the entity as a mobidik
//            std::string mobidikIDNew = mobidikId.str() + "-Mobidik";
//            req.setExistenceProbability ( mobidikIDNew, 1.0 ); // TODO magic number
//            req.setConvexHullNew ( mobidikIDNew, chull, mobidikPose, lastUpdateTimestamp, "/map");

           req.setProperty ( mobidikEntity->id(), featureProperties, mobidikFeatures );
           mobidikFeatures_ = mobidikFeatures;
            req.setFlag(mobidikEntity->id(), "Mobidik"); // TODO update while moving backwards with sensor at the back!
            //req.setFlag(mobidikEntity->id(), "locked"); // TODO update while moving backwards with sensor at the back! this prevents updates from the tracking part!! TODO THis gives problems!!
           std::cout<< "Requested to update properties of entity with id = " << mobidikEntity->id() << std::endl;
//                      mobidikFeatures.printProperties(); 
                      std::cout << "getSetpointInFrontOfMobidik: id = " << mobidikEntity->id() << std::endl;
//            req.removeEntity(mobidikId);
            std::cout << "end of setMobidikPosition" << std::endl;
            
            return true;
//         }

//     }

//     return false;

};  


bool MobidikCollection::updateMobidikPosition ( const ed::WorldModel& world,ed::UpdateRequest& req, ed::UUID mobidikID, visualization_msgs::Marker* points ) 
// Store mobidik in ED as it might not be detected anymore when the robot rotates
{
  
  ed::EntityConstPtr mobidikEntity;
  if(!getEntityPointer(world, mobidikID, mobidikEntity))
  {
          ROS_WARN("mobidik entity not found");
          return false;
  }
  
//   std::cout << "setMobidikPosition"<< std::endl;
  double mobidikWidth, mobidikLength, MD_yaw, lastUpdateTimestamp;    
  ed::tracking::FeatureProperties mobidikFeatures;    

//   std::cout << "mobidikIDPtr = " << mobidikEntity<< std::endl;
//   std::cout <<"MobidikID = " << mobidikEntity->id() << std::endl;
                mobidikFeatures = mobidikEntity->property ( featureProperties );
                ed::tracking::Rectangle mobidikModel = mobidikFeatures.getRectangle();
//   std::cout << "mobidikFeatures are: " << std::endl;
// mobidikFeatures.printProperties();

//                 std::cout <<"xPos = " << mobidikModel.get_x()  << std::endl; 
                geo::Vec3d origin ( mobidikModel.get_x(), mobidikModel.get_y(), mobidikModel.get_z() );
                
                mobidikWidth = mobidikModel.get_w();
                mobidikLength = mobidikModel.get_d();

                MD_yaw = mobidikModel.get_yaw();
                wrap2twopi ( &MD_yaw );
                
//                 std::cout << "Mobidik yaw = " << MD_yaw << std::endl;
                
                lastUpdateTimestamp = mobidikEntity->lastUpdateTimestamp();
                
// mobidikFeatures.printProperties();

        std::string orientationWPID = orientationWPID_;
std::cout << "orientationWPID = " << orientationWPID << std::endl;
  ed::EntityConstPtr wpEntity;
  if(!getEntityPointer(world, orientationWPID, wpEntity))
  {
          ROS_WARN("orient_wp_ not found");
          return false;
  }

            geo::Pose3D poseWP = wpEntity.get()->pose();
            geo::Quaternion rotationWP = poseWP.getQuaternion();

            tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
            tf::Matrix3x3 matrix ( q );
            double WP_roll, WP_pitch, WP_yaw;
            matrix.getRPY ( WP_roll, WP_pitch, WP_yaw );
//             std::cout << "WP_yaw before = " << WP_yaw << std::endl;
            wrap2twopi ( &WP_yaw );
            
//             std::cout << "WP_yaw = " << WP_yaw << std::endl;

            // find quadrant closest to the orientation of the mobidik
            geo::real diffAngleMin = INFINITY;
            double angle, angleAtMinDiff, deltaAngleCorrected;
            for ( unsigned int ii = 0; ii < 4; ii++ )
            {
                double deltaAngle = ii*M_PI_2;
                angle = MD_yaw + deltaAngle;
                wrap2twopi ( &angle );

                geo::real diffAngle = std::min ( std::fabs ( angle - WP_yaw ), std::fabs ( angle + 2*M_PI - WP_yaw ) );
                diffAngle = std::min ( diffAngle, std::fabs ( angle - 2*M_PI - WP_yaw ) );
                
                if ( diffAngle < diffAngleMin )
                {
                    diffAngleMin = diffAngle;
                    angleAtMinDiff = angle;
                    deltaAngleCorrected = deltaAngle;
                }
            }
            
            MD_yaw = angleAtMinDiff;
            if ( deltaAngleCorrected == M_PI_2 || deltaAngleCorrected == 3*M_PI_2 ) // change of pi/2 of 3*pi/2-> width and length change
            {
                double mobidikWidthOld = mobidikWidth;
                mobidikWidth = mobidikLength;
                mobidikLength = mobidikWidthOld;
            }

            geo::Mat3 rotation;
            rotation.setRPY ( 0.0, 0.0, MD_yaw );
//             mobidikPose.setBasis ( rotation );

            ed::ConvexHull chull;
            
            float length = std::sqrt ( std::pow (0.5* mobidikWidth, 2.0 ) + std::pow ( 0.5*mobidikLength, 2.0 ) );
// std::cout << "mobidik pose = " << mobidikPose.getOrigin().getX() << mobidikPose.getOrigin().getY() << std::endl;
            geometry_msgs::Point p;
            angle = MD_yaw + atan2(mobidikLength, mobidikWidth);
            geo::Vec2f point ( mobidikModel.get_x() + length*cos ( angle ), mobidikModel.get_y() + length*sin ( angle )  );
            chull.points.push_back ( point ); 
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
          
            angle = MD_yaw + atan2(mobidikLength, -mobidikWidth);
            point.x = mobidikModel.get_x() + length*cos ( angle ); point.y =  mobidikModel.get_y() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
            
            angle = MD_yaw + atan2(-mobidikLength, -mobidikWidth);
            point.x = mobidikModel.get_x() + length*cos ( angle ); point.y =  mobidikModel.get_y() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p ); 
            
            angle = MD_yaw + atan2(-mobidikLength, mobidikWidth);
            point.x = mobidikModel.get_x() + length*cos ( angle ); point.y =  mobidikModel.get_y() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
            
            mobidikFeatures.rectangle_.set_w(mobidikWidth);
            mobidikFeatures.rectangle_.set_d(mobidikLength);
//             std::cout << "Before setting new properties." << std::endl;

            mobidikFeatures.rectangle_.set_yaw(MD_yaw);

// std::cout << "MD_yaw set at " << MD_yaw << std::endl;
            

           req.setProperty ( mobidikEntity->id(), featureProperties, mobidikFeatures );
           mobidikFeatures_ = mobidikFeatures;
           req.setFlag(mobidikEntity->id(), "Mobidik"); // TODO update while moving backwards with sensor at the back!
          // req.setFlag(mobidikEntity->id(), "locked"); // TODO update while moving backwards with sensor at the back! this prevents updates from the tracking part!! TODO THis gives problems!!
//            std::cout<< "Requested to update properties of entity with id = " << mobidikEntity->id() << std::endl;
//                       mobidikFeatures.printProperties(); 
//                       std::cout << "getSetpointInFrontOfMobidik: id = " << mobidikEntity->id() << std::endl;
//             std::cout << "end of setMobidikPosition" << std::endl;
            
            return true;

};  




// bool MobidikCollection::getMobidikPosition( const ed::WorldModel& world, ed::UUID mobidikID, geo::Pose3D *mobidikPose )
// {
//     for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
//     {
//         const ed::EntityConstPtr& e = *it;
// 
//         if ( mobidikID.str().compare ( e.get()->id().str() )  == 0 )
//         {
//                 ed::tracking::FeatureProperties properties = e->property ( featurePropertiesKey ); // Improve! pose should not be communicated via the featureproperties, but via e.get()->pose();
//                 geo::Vec3d origin ( properties.getRectangle().get_x(), properties.getRectangle().get_y(), 0 );
//                 
//                 geo::Mat3 rotation;
//                 rotation.setRPY ( 0.0, 0.0, properties.getRectangle().get_yaw() );
//                 
//                 mobidikPose->setOrigin( origin );
//                 mobidikPose->setBasis ( rotation );
// 
//                 return true;
//         }
//     }
// 
//     return false;
// }

bool MobidikCollection::getEntityPointer(const ed::WorldModel& world, ed::UUID MobidikID_ED, ed::EntityConstPtr& entityPointer)
{
        for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
        {
                const ed::EntityConstPtr& e = *it;
                if ( MobidikID_ED.str().compare ( e.get()->id().str() )  == 0 )
                {
//                         std::cout << "MobidikID new = " << e->id()  << " pointer = " << e << std::endl; // TODO THIS ONE CHANGED!!!!
                        entityPointer = e;
                        return true;
                }
        }
        
        return false;
}

bool MobidikCollection::getSetpointInFrontOfMobidik ( const ed::WorldModel& world,  ed::UpdateRequest& req, const ed::UUID mobidikID, geo::Pose3D *setpoint, visualization_msgs::Marker* points, float distance)
{
std::cout << "getSetpointInFrontOfMobidik: id = " << mobidikID << std::endl;
 //   geo::Pose3D mobidikPose;

        ed::EntityConstPtr mobidikEntity;
       if( getEntityPointer(world, mobidikID, mobidikEntity) )
       {
    
                float mobidikLength;
                ed::tracking::FeatureProperties entityProperties;
                if( mobidikEntity->property ( featureProperties ))
                {
//                         entityProperties = mobidikEntity->property ( featureProperties );
                         entityProperties = mobidikFeatures_;
                        std::cout << "getSetpointInFrontOfMobidik" << std::endl;
//                         entityProperties.printProperties();
                        
                        mobidikLength = entityProperties.rectangle_.get_w();
                } else
                {
                        mobidikLength = MOBIDIK_LENGTH; 
                }
          /*  mobidikPose =  e.get()->pose();
std::cout << mobidikPose.getOrigin().getX() << ", " << mobidikPose.getOrigin().getY() << std::endl;
            float dist = 0.5* ( ROPOD_LENGTH + mobidikLength ) + DIST_IN_FRONT_OFF_MOBID;
std::cout << "dist = " << dist << std::endl;
            *setpoint = mobidikPose;
            
            geo::Quaternion rotation = mobidikPose.getQuaternion();
            tf::Quaternion q ( rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW() );
            tf::Matrix3x3 matrix ( q );
            double roll, pitch, yaw;

std::cout << "yaw = " << yaw << std::endl;
            matrix.getRPY ( roll, pitch, yaw );
            */
       /*     
            geo::Vec3d origin ( , entityProperties.getRectangle().get_y(), 0 );
                
                geo::Mat3 rotation;
                rotation.setRPY ( 0.0, 0.0, entityProperties.getRectangle().get_yaw() );
                
                mobidikPose->setOrigin( origin );
                mobidikPose->setBasis ( rotation );
                *setpoint = mobidikPose;
         */   
            std::cout << "mobidikLength = " << mobidikLength << std::endl;
            float dist = 0.5* ( ROPOD_LENGTH + mobidikLength ) + distance;
            double yaw = entityProperties.getRectangle().get_yaw();
            
            std::cout << "yaw = " << yaw << std::endl;
            std::cout << "dist = " << dist << std::endl; 
            
             geo::Mat3 rotation;
                rotation.setRPY ( 0.0, 0.0, entityProperties.getRectangle().get_yaw() );
                setpoint->setBasis ( rotation );
            
            geo::Vec3d origin ( entityProperties.getRectangle().get_x() + dist*cos ( yaw ), entityProperties.getRectangle().get_y() + dist*sin ( yaw ), 0.0 );
            setpoint->setOrigin ( origin );
            geometry_msgs::Point p;
            p.x = setpoint->getOrigin().getX();
            p.y = setpoint->getOrigin().getY();
            p.z = 0.0;
            
// geo::Vec3d originMD (entityProperties.getRectangle().get_x(), entityProperties.getRectangle().get_y(), 0.0);
// mobidikPos->setOrigin( originMD );
// mobidikPos->setRPY (0.0, 0.0, entityProperties.getRectangle().get_yaw() );

            std::cout << "Point = " << p << std::endl;

            points->points.push_back ( p );
            
//             req.removeFlag(mobidikEntity->id(), "locked");
            return true;
//         }
      }

    return false;
};


/*--------------------------------------------------------*/
// void MobidikCollection::startNavigation(wm::Elevator &elevator,nav_msgs::Path Pathmsg)
// {
//     resetNavigation();
//     route_busy_ = true;
//     // if we get elevator poses from ropod_wm_mediator, use them
//     if (Pathmsg.poses.size() == 3)
//     {
//         elevator.setInsideElevatorPose(wm::getWMPose(Pathmsg.poses[0]), wm::getWMPose(Pathmsg.poses[1]));
//         elevator.setOutsideElevatorPose(wm::getWMPose(Pathmsg.poses[2]));
//         ROS_INFO("Poses from elevator message assigned: %f", elevator.wayp1_elevator.position.x);
//     }
//     ROS_INFO("start navigation trough elevator");
//     return;
// }

/*--------------------------------------------------------*/
void MobidikCollection::pauseNavigation()
{
    nav_paused_req_ = true;
    nav_state_bpause_ = nav_state_;
    nav_next_state_ = MOBID_COLL_NAV_PAUSED;
    ROS_INFO("Navigation paused");
    return;
};

/*--------------------------------------------------------*/
void MobidikCollection::resumeNavigation()
{
    nav_paused_req_ = false;
    if(nav_state_bpause_ == MOBID_COLL_NAV_BUSY)
        nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
    else
        nav_next_state_ = nav_state_bpause_;
    ROS_INFO("Navigation resumed");
    return;
}

/*--------------------------------------------------------*/
void MobidikCollection::resetNavigation()
{
    base_position.reset();
    route_busy_ = false;
    nav_paused_req_ = false;
    waypoint_cnt_ = 0;
    nav_state_ = MOBID_COLL_NAV_IDLE;
    nav_next_state_ = MOBID_COLL_NAV_IDLE;
    nav_state_bpause_ = MOBID_COLL_NAV_IDLE;
    nav_next_state_wp_ = MOBID_COLL_NAV_IDLE;
    nav_state_release_ = MOBID_REL_NAV_IDLE;
    nav_next_state_release_ = MOBID_REL_NAV_IDLE;
    nav_next_state_wp_release_ = MOBID_REL_NAV_IDLE;
    
    ROS_INFO("Navigation reset");
}

/*--------------------------------------------------------*/
void MobidikCollection::stopNavigation()
{
    base_position.reset();
    route_busy_ = false;
    waypoint_cnt_ = 0;
    ROS_INFO("Navigation stopped");
}

/*--------------------------------------------------------*/
bool MobidikCollection::isPositionValid()
{
    if(base_position)
        return true;
    else
        return false;
}

void MobidikCollection::initAvgWrench(geometry_msgs::WrenchStamped *wrench)
{
        wrench->wrench.force.x = INFINITY;
        wrench->wrench.force.y = INFINITY;
        wrench->wrench.force.z = INFINITY;
        wrench->wrench.torque.x = INFINITY;
        wrench->wrench.torque.y = INFINITY;
        wrench->wrench.torque.z = INFINITY;
}

void MobidikCollection::initNavState()
{
std::cout << "initNavSTate" << std::endl;
        nav_state_ = MOBID_COLL_FIND_MOBIDIK;
        avgWrenchesDetermined_ = false;
        initAvgWrench(&avgWrenches_.front);
        initAvgWrench(&avgWrenches_.left);
        initAvgWrench(&avgWrenches_.back);
        initAvgWrench(&avgWrenches_.right);
        xy_goal_tolerance_  = GOAL_MOBID_COLL_REACHED_DIST;
        yaw_goal_tolerance_ = GOAL_MOBID_REACHED_ANG;

        nav_state_ = MOBID_COLL_FIND_MOBIDIK;
std::cout << "initnavSTate succeeded" << std::endl;
}

void MobidikCollection::initNavStateRelease()
{
std::cout << "initNavSTateRel" << std::endl;
        nav_state_release_ = MOBID_REL_GET_SETPOINT_FRONT;
        avgWrenchesDetermined_ = false;
        initAvgWrench(&avgWrenches_.front);
        initAvgWrench(&avgWrenches_.left);
        initAvgWrench(&avgWrenches_.back);
        initAvgWrench(&avgWrenches_.right);
        xy_goal_tolerance_  = GOAL_MOBID_LOAD_REACHED_DIST; // we start with lower tolerance
        yaw_goal_tolerance_ = GOAL_MOBID_REACHED_ANG_UNDOCK;        
std::cout << "initNavSTateRel succeeded" << std::endl;
}

geometry_msgs::WrenchStamped MobidikCollection::determineAvgWrench(std::vector<geometry_msgs::WrenchStamped> wrenchVector)
{
        float sumForcex = 0.0, sumForcey = 0.0, sumForcez = 0.0, sumTorquex = 0.0, sumTorquey = 0.0, sumTorquez = 0.0;
            for ( unsigned int ii = 0; ii < wrenchVector.size(); ii++ )
            {
                sumForcex += wrenchVector[ii].wrench.force.x;
                sumForcey += wrenchVector[ii].wrench.force.y;
                sumForcez += wrenchVector[ii].wrench.force.z;
                sumTorquex += wrenchVector[ii].wrench.torque.x;
                sumTorquey += wrenchVector[ii].wrench.torque.y;
                sumTorquez += wrenchVector[ii].wrench.torque.z;
            }

            geometry_msgs::WrenchStamped avgWrench;
            avgWrench.wrench.force.x = sumForcex / wrenchVector.size();
            avgWrench.wrench.force.y = sumForcey / wrenchVector.size();
            avgWrench.wrench.force.z = sumForcez / wrenchVector.size();
            avgWrench.wrench.torque.x = sumTorquex / wrenchVector.size();
            avgWrench.wrench.torque.y = sumTorquey / wrenchVector.size();
            avgWrench.wrench.torque.z = sumTorquez/ wrenchVector.size();
            
            return avgWrench;            
}

void MobidikCollection::printWrenches(geometry_msgs::WrenchStamped wrench)
{
        std::cout <<
        wrench.wrench.force.x << ", " << 
        wrench.wrench.force.y << ", " << 
        wrench.wrench.force.z << ", " << 
        wrench.wrench.torque.x << ", " << 
        wrench.wrench.torque.y << ", " << 
        wrench.wrench.torque.z << std::endl;
}

void MobidikCollection::determineAvgWrenches()
{
        std::vector<geometry_msgs::WrenchStamped> wrenchesFront, wrenchesLeft, wrenchesBack, wrenchesRight;
        for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ )
        {
                wrenchesFront.push_back( bumperWrenchesVector_[ii].front );
                wrenchesLeft.push_back( bumperWrenchesVector_[ii].left );
                wrenchesBack.push_back( bumperWrenchesVector_[ii].back );
                wrenchesRight.push_back( bumperWrenchesVector_[ii].right );
        }
        
        avgWrenches_.front = determineAvgWrench( wrenchesFront );
        avgWrenches_.left = determineAvgWrench( wrenchesLeft );
        avgWrenches_.back = determineAvgWrench( wrenchesBack );
        avgWrenches_.right = determineAvgWrench( wrenchesRight );
        
/*        printWrenches(avgWrenches_.front);
        printWrenches(avgWrenches_.left);
        printWrenches(avgWrenches_.back);
        printWrenches(avgWrenches_.right);
*/
        
        avgWrenchesDetermined_ = true;
}

// void MobidikCollection::initRelState()
// {
//         nav_state_release_ = MOBID_REL_GET_SETPOINT_FRONT;
// }

/*--------------------------------------------------------*/
bool MobidikCollection::isWaypointAchieved(double& dist_tolerance, double& angle_tolerance)
{

    // In elevator we checked whether teh specific position is reached.
    tf::Quaternion qtemp = tf::Quaternion(base_position->pose.orientation.x, base_position->pose.orientation.y, 
                                          base_position->pose.orientation.z, base_position->pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position->pose.position.x, base_position->pose.position.y, 0.0);
    base_positiontf_ = tf::Transform( qtemp, v3temp);
    qtemp = tf::Quaternion(goal_.target_pose.pose.orientation.x, goal_.target_pose.pose.orientation.y,
                           goal_.target_pose.pose.orientation.z,goal_.target_pose.pose.orientation.w);
    v3temp = tf::Vector3(goal_.target_pose.pose.position.x,goal_.target_pose.pose.position.y, 0.0);
    waypoint_tf_ = tf::Transform( qtemp, v3temp);

    tf::Transform diff_tf = base_positiontf_.inverseTimes(waypoint_tf_);
    v3temp = diff_tf.getOrigin();
    qtemp = diff_tf.getRotation();
    
    std::cout << "Is waypoint achieved: diff_tf origin = " << v3temp << " & rot = " << qtemp << std::endl;
    
    if (pow( v3temp.x(),2) + pow(v3temp.y(),2) < pow(dist_tolerance,2)
            && fabs(qtemp.getAngle()) < angle_tolerance)
        return true;
    else
        return false;
}


/*--------------------------------------------------------*/
TaskFeedbackCcu MobidikCollection::callNavigationStateMachine(ros::Publisher &nav_cancel_pub, maneuver_navigation::Goal &mn_goal, bool& sendgoal, visualization_msgs::MarkerArray markerArray,
                                                              std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest, 
                                                              std_msgs::UInt16* controlMode, ros::Publisher &cmv_vel_pub, ropodNavigation::wrenches bumperWrenches, const bool robotReal,
                                                              ros::Publisher &dockingCommand, ropod_ros_msgs::DockingFeedback dockingFeedback, bool sensorBack)
{
std::cout << "mobidikColl state machine start" << std::endl;
std::cout << "nav_state = " << nav_state_ << std::endl;   

//printFlags
/*
        ed::EntityConstPtr mobidikEntity;
       if( getEntityPointer(world, MobidikID_ED, mobidikEntity) )
       {
               mobidikEntity->printFlags();
       }
*/
TaskFeedbackCcu tfb_nav;
    tfb_nav.wayp_n = waypoint_cnt_;
    tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;
    visualization_msgs::Marker marker;
    bool idUpdated = false;
//std::cout << "debug1"<< std::endl;
    
    tf::Quaternion q;
    geometry_msgs::Twist output_vel;
    output_vel.angular.x = 0.0;
    output_vel.angular.y = 0.0;
    output_vel.angular.z = 0.0;
    output_vel.linear.x = 0.0;
    output_vel.linear.y = 0.0;
    output_vel.linear.z = 0.0;
    
    float avgForce, avgTorque;
    bool touched, forceCheck, torqueCheck;
//std::cout << "debug2"<< std::endl;
    
    tf::Pose base_position_pose;
    double robot_yaw;
    
    geo::Pose3D mobidikPose;
    double dist2;
    ed::tracking::FeatureProperties mobidikProperties;
    float mobidikLength;
    ed::EntityConstPtr mobidikPointer;
    
    int sysCommand;
  // std::cout << "debug3"<< std::endl; 
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
//std::cout << "debug4"<< std::endl;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
//std::cout << "debug5"<< std::endl;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
//std::cout << "debug6"<< std::endl;
    
     // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
//std::cout << "debug7"<< std::endl;

    mn_goal.conf.precise_goal = true;
//std::cout << "debug7.1"<< std::endl;

    mn_goal.conf.use_line_planner = true;
//std::cout << "debug7.2"<< std::endl;
    
    mn_goal.conf.append_new_maneuver =  false;
    
    mn_goal.start.pose = base_position->pose; // always plan from current pose
//std::cout << "debug8"<< std::endl;
    
    mn_goal.goal.header.frame_id = "map";
    mn_goal.goal.header.stamp = ros::Time::now();
    mn_goal.start.header.frame_id = "map";
    mn_goal.start.header.stamp = ros::Time::now();
    
    ropod_ros_msgs::DockingCommand dockingMsg;
    
    std::cout << "Variable initiated, going 2 state machine" << std::endl;

    switch(nav_state_)
    { 
            
    case MOBID_COLL_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
        ROS_INFO("NAV_IDLE");
        break;
        
    case MOBID_COLL_FIND_MOBIDIK:
            ROS_INFO("MOBID_COLL_FIND_MOBIDIK");
            
            // Configure holonomic robot to move more accurately towards target
            sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_y 0.5 &");
            sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS weight_kinematics_nh 0 &");
            sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS weight_kinematics_forward_drive 0 &");
            
            if(  getMobidik( world, &MobidikID_ED ) ) // Assumption: there is only 1 mobidik available at the moment in the area
            {           
                    std::cout << "MobidikID_ED_ = " << MobidikID_ED << std::endl;
                    //setMobidikPosition ( world, req, areaID, marker, &MobidikID_ED_, &points ); 
                    if( setMobidikPosition ( world, req, areaID, MobidikID_ED, &points ) )
                    {  
                        markerArraytest->markers.push_back( points );
                        nav_next_state_ = MOBID_COLL_FIND_SETPOINT_FRONT;
                        ROS_INFO ( "Mobidik Collection: Mobidik found" );
                        tfb_nav.fb_nav = MOBIDIK_DETECTED;
            
                    }
                    else
                    {
                            std::cout << "MobidikID_ED_ = " << MobidikID_ED << std::endl;
                            std::cout << "areaID = " << areaID << std::endl;
                            
                        nav_next_state_ = MOBID_COLL_FIND_MOBIDIK;
                        ROS_WARN("Mobidik Collection: No mobidik found"); // TODO Recovery behaviour
                        tfb_nav.fb_nav = NO_MOBIDIK_DETECTED;
                    }
             }
            else 
            {
                    nav_next_state_ = MOBID_COLL_NAV_IDLE;
                    ROS_WARN("Mobidik Collection: No mobidik found in get mobidik"); // TODO Recovery behaviour
                    tfb_nav.fb_nav = NO_MOBIDIK_DETECTED;
            }              
         break;
         
    case MOBID_COLL_FIND_SETPOINT_FRONT:
            ROS_INFO("MOBID_COLL_FIND_SETPOINT_FRONT");         
             
            if( !getSetpointInFrontOfMobidik ( world, req, MobidikID_ED, &setpoint_, &points, DIST_IN_FRONT_OFF_MOBID) ) // WM updated?
            {
                    nav_next_state_ = MOBID_COLL_NAV_IDLE; 
                    ROS_WARN("Mobidik Collection: No mobidik-entity found"); // TODO Recovery behaviour
                    tfb_nav.fb_nav = NO_MOBIDIK_DETECTED;
                    break;
            }
            
            req.setFlag(MobidikID_ED, "locked"); // In order to prevent potential problems with partial updates. Release when the mobidik is entirely seen at the back.            
            point2goal(&setpoint_);
            markerArraytest->markers.push_back ( points );           
            nav_next_state_wp_ = MOBID_COLL_NAV_CONNECTING;
            nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;            
            bumperWrenchesVector_.clear();
            break;
       
        
    case MOBID_COLL_NAV_CONNECTING:
        ROS_INFO ( "MOBID_COLL_NAV_CONNECTING" );

        
        req.removeFlag(MobidikID_ED, "locked");

        touched = false;

//         if ( ! robotReal )
//         {
        
//             getMobidikPosition ( world, MobidikID_ED, &mobidikPose );
        
        
        // TODO: get current mobidikPosition
        // get error with respect to setpoint. Setpoint is a certain distance in front of the mobidik.
        // move with velocity set in BACKWARD_VEL_DOCKING, projected on the error (so the error scales with the error in both directions)
        // stop when we are close
        
        
            controlMode->data = ropodNavigation::LLC_DOCKING;
            
            if( !getEntityPointer(world, MobidikID_ED, mobidikPointer) )
            {
                    ROS_WARN("Mobidik Collection: No mobidik-entity found in MOBID_COLL_NAV_CONNECTING");
            }
            else
            {
                    if( sensorBack)
                    {
                            updateMobidikPosition( world, req, MobidikID_ED, &points );
                    }

                        geo::Pose3D setPointOnMobidik; 
                        if( !getSetpointInFrontOfMobidik ( world, req, MobidikID_ED, &setPointOnMobidik, &points, 0.0) ) // WM updated?
                        {
                                nav_next_state_ = MOBID_COLL_NAV_IDLE; 
                                ROS_WARN("Mobidik Connection: No mobidik-entity found"); // TODO Recovery behaviour
                                tfb_nav.fb_nav = NO_MOBIDIK_DETECTED;
                                break;
                        }
                        tfb_nav.fb_nav = MOBIDIK_DETECTED;
               /*         
        //                 mobidikProperties = mobidikPointer->property ( featureProperties );
                        mobidikProperties = mobidikFeatures_;
        //                 std::cout << "ID reference " << mobidikPointer->id() << std::endl;R
        //                 std::cout << "MobidikID_ED = " << MobidikID_ED << std::endl;
                        mobidikLength = mobidikProperties.rectangle_.get_w();
                        
                        // Transform error from world frame to robot frame                        
                        tf::Quaternion q(base_position->pose.orientation.x, base_position->pose.orientation.y, base_position->pose.orientation.z, base_position->pose.orientation.w);
                        tf::Matrix3x3 robotM(q);
                        double robotRoll, robotPitch, robotYaw;
                        robotM.getRPY(robotRoll, robotPitch, robotYaw);
                        
                        geo::Vec2f errorWorld, errorRobot; // world or robot coordinates
                        errorWorld.x = setPointOnMobidik.getOrigin().getX() - base_position->pose.position.x;
                        errorWorld.y = setPointOnMobidik.getOrigin().getY() - base_position->pose.position.y;
                        
                        errorRobot.x = errorWorld.x*cos(-robotYaw) - errorWorld.y*sin(-robotYaw);
                       // errorRobot.x = errorRobot.x - sgn( errorRobot.x ) * 0.5*( ROPOD_LENGTH );
                        errorRobot.y = errorWorld.x*sin(-robotYaw) + errorWorld.y*cos(-robotYaw);
                        float rotError = mobidikProperties.rectangle_.get_yaw() - robotYaw;
                
                        std::cout << "Setpoint on mobidik: x, y = " << setPointOnMobidik.getOrigin().getX()  << ", " << setPointOnMobidik.getOrigin().getY() << std::endl;
                
                        dist2 = std::pow ( errorWorld.x , 2.0 ) + std::pow ( errorWorld.y , 2.0 );


                        std::cout << "base_position_->pose.position.x, y = " << base_position->pose.position.x << ", " << base_position->pose.position.y << std::endl;
                        // std::cout << "mobidikPose.getOrigin().getX(), y = " << mobidikPose.getOrigin().getX() << ", " << mobidikPose.getOrigin().getY() << std::endl;
                        std::cout << "mobidikProperties.rectangle_.get_x(), y = " << mobidikProperties.rectangle_.get_x() << ", " << mobidikProperties.rectangle_.get_y() << std::endl;
//                         std::cout << "mobidikLength = " << mobidikLength << std::endl;

//                         std::cout << "dist ref = " << std::pow ( 0.5* ( ROPOD_LENGTH + mobidikLength ) + DIST_CONN_SIM, 2.0 ) << std::endl;
                        
                        std::cout << "errorRobot = " << errorRobot.x << ", " << errorRobot.y << std::endl;

                        float errorNorm = std::sqrt( dist2 );
                        
                       std::cout << "error = " << errorNorm << std::endl;
                        
                        geo::Vec2f errorNormalized;
                        errorNormalized.x = errorRobot.x/errorNorm*BACKWARD_VEL_DOCKING;
                        errorNormalized.y = errorRobot.y/errorNorm*BACKWARD_VEL_DOCKING;
                        
//                         if(std::fabs(errorRobot.x) < std::fabs(errorNormalized.x) )
//                         {
//                                 output_vel.linear.x = errorRobot.x;
//                         }
//                         else{
                                output_vel.linear.x = errorNormalized.x;
//                         }
                        
//                         if(std::fabs(errorRobot.y) < std::fabs( errorNormalized.y ) )
//                         {
//                                 output_vel.linear.y = errorRobot.y;
//                         }
//                         else{
                                output_vel.linear.y = errorNormalized.y;
//                         }
                        
//                         if(std::fabs(rotError) < std::fabs( MAX_ROT_VEL_DOCKING ) )
//                         {
//                                 output_vel.angular.z = 0.1*rotError;
//                         }
//                         else{
//                                 output_vel.angular.z = sgn(rotError)*MAX_ROT_VEL_DOCKING;
//                         }
                        
                        //output_vel.linear.x = std::min(  errorRobot.x, errorNormalized.x ); // For both situations (with/without mobidik updates) similar?
                        //output_vel.linear.y = std::min( errorRobot.y, errorNormalized.y ); // TODO prevent constantly going backwards if there are problems. Check if mobidik is still there!
                        //output_vel.angular.z = std::min(rotError, (float) MAX_ROT_VEL_DOCKING);
                        cmv_vel_pub.publish ( output_vel );
                        
                        std::cout << "Output vel x, y = " <<  output_vel.linear.x << ", " << output_vel.linear.y << std::endl;

//                         if ( dist2 < std::pow ( 0.5* ( ROPOD_LENGTH + mobidikLength ) + DIST_CONN_SIM, 2.0 ) )
                        
                        if( fabs(errorRobot.x) < DIST_CONN_X && fabs(errorRobot.y) < DIST_CONN_Y ) // TODO tuning of DIST_CONN_X & DIST_CONN_Y?
                        {
                                  ROS_INFO ( "Touched = true" );
                                touched = true;
                        }
                        
//                         if ( dist2 < std::pow ( 0.5* ( ROPOD_LENGTH ) + DIST_CONN_SIM, 2.0 ) )
//                         {
//                                 touched = true;
//                         }
*/
              
               sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_y 0.2 &");
               sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_x 0.2 &");
                           
               xy_goal_tolerance_  = GOAL_MOBID_COLL_REACHED_DIST; // for the last part we decrease tolerance
               yaw_goal_tolerance_ = GOAL_MOBID_REACHED_ANG; 
               
               setpoint_ = setPointOnMobidik;
               point2goal(&setpoint_);          
               
               nav_next_state_wp_ = MOBID_COLL_INIT_COUPLING;
               nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;            
               
        //         }
        //         else
        //         {
        // 
        //             bumperWrenchesVector_.push_back ( bumperWrenches );
        //             if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES )
        //             {
        // 
        //                 bumperWrenchesVector_.erase ( bumperWrenchesVector_.begin() );
        // 
        //                 if ( !avgWrenchesDetermined_ )
        //                 {
        //                     determineAvgWrenches();
        // 
        //                 }
        //                 else
        //                 {
        // 
        //                     controlMode->data = ropodNavigation::LLC_DOCKING;
        //                     output_vel.linear.x = -BACKWARD_VEL_DOCKING;
        //                     cmv_vel_pub.publish ( output_vel );
        // 
        //                     avgForce = 0.0;
        //                     avgTorque = 0.0;
        //                     for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ ) // TODO can be more efficient by storing the sum
        //                     {
        //                         avgForce += bumperWrenchesVector_[ii].back.wrench.force.x;
        //                         avgTorque += bumperWrenchesVector_[ii].back.wrench.torque.z;
        //                     }
        // 
        //                     avgForce /= bumperWrenchesVector_.size();
        //                     avgTorque /= bumperWrenchesVector_.size();
        //                     
        //                     forceCheck = std::fabs ( avgForce - avgWrenches_.back.wrench.force.x ) > MIN_FORCE_TOUCHED;
        //                     torqueCheck = std::fabs ( avgTorque - avgWrenches_.back.wrench.torque.z ) < MAX_TORQUE_TOUCHED;
        // 
        //                     if ( forceCheck && torqueCheck )
        //                     {
        //                         touched = true;
        //                     }
        //                 }
        //             }
        //         }

//                         if ( touched )
//                         {
//                                 ROS_INFO ( "End of connecting state. Going to dock now." );
                //             if ( ! robotReal )
                //             {
                //                 tfb_nav.fb_nav = NAV_DOCKED;
                //                 nav_next_state_ = MOBID_COLL_NAV_EXIT_COLLECT_AREA;
                //                 ROS_WARN ( " You are in simulation mode. The mobidik is assumed to be connected now." );
                //             }  else {
                //                 bumperWrenchesVector_.clear();
                //                 avgWrenchesDetermined_ = false;
                //                 controlMode->data = ropodNavigation::LLC_VEL; //LLC_NORMAL
                                
//                                 dockingMsg.docking_command = DOCKING_COMMAND_DOCK;
//                                 dockingCommand.publish( dockingMsg );
//                                 nav_next_state_  = MOBID_COLL_NAV_COUPLING;
                //                 
                //             }
//                                 stamp_start_ = ros::Time::now();
//                                 stamp_wait_ = ros::Duration(TIME_WAIT_CHANGE_OF_FOOTPRINT);         
//                         }
                }
        break;
        
        
    case MOBID_COLL_INIT_COUPLING:
            ROS_INFO ( "MOBID_COLL_INIT_COUPLING" );
                 dockingMsg.docking_command = DOCKING_COMMAND_DOCK;
                 dockingCommand.publish( dockingMsg );
                 nav_next_state_  = MOBID_COLL_NAV_COUPLING;
                 
                 stamp_start_ = ros::Time::now();
                 stamp_wait_ = ros::Duration(TIME_WAIT_CHANGE_OF_FOOTPRINT);      
        break;
         
    case MOBID_COLL_NAV_COUPLING: // TODO
        ROS_INFO ( "MOBID_COLL_NAV_COUPLING" );
        controlMode->data = ropodNavigation::LLC_VEL;
        // couple mobidik manually and wait for signal;
	output_vel.linear.x = 0.0; // TODO set wheel controllers of?
        cmv_vel_pub.publish ( output_vel );
        touched = false;
        
/*        bumperWrenchesVector_.push_back ( bumperWrenches );
        if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES ) 
        {
            bumperWrenchesVector_.erase ( bumperWrenchesVector_.begin() ); // just to be sure
            if ( !avgWrenchesDetermined_ )
            {
                determineAvgWrenches();

            }
            else
            {

                avgForce = 0.0;
                for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ )
                {
                    avgForce += bumperWrenchesVector_[ii].front.wrench.force.x;
                }

                avgForce /= bumperWrenchesVector_.size();
                
                forceCheck = std::fabs ( avgForce - avgWrenches_.front.wrench.force.x ) > MIN_FORCE_TOUCHED;

                if ( forceCheck )
                {
                    touched = true;
                }
            }
        }
*/

        if ( !robotReal )
        {
                ros::Duration diff = ros::Time::now() -stamp_start_;
                if ( diff.toSec() > 5.0 )
                {
                         touched = true;
                          ROS_WARN ( " You are in simulation mode. The mobidik is assumed to be connected now." );
                }

        }
        else
        {
                if( dockingFeedback.docking_status == DOCKING_FB_DOCKED )
                {
                        touched = true;
                        tfb_nav.fb_nav = COUPLING_SUCCEEDED;
                } else if (dockingFeedback.docking_status == DOCKING_FB_REST) { //TODO check if it is in init state
                	tfb_nav.fb_nav = COUPLING_FAILED;
                }
        }

        if ( touched )
        {
            // Docking done!
           ROS_INFO("MOBIDIK COUPLED");
                                        
            tfb_nav.fb_nav = NAV_DOCKED;
           // req.removeFlag(MobidikID_ED, "Mobidik");
            req.removeFlag(MobidikID_ED, "locked"); // TODO update while moving backwards with sensor at the back!
            nav_next_state_ = MOBID_COLL_NAV_EXIT_COLLECT_AREA;
            bumperWrenchesVector_.clear();
            stamp_start_ = ros::Time::now();
            stamp_wait_ = ros::Duration(TIME_WAIT_CHANGE_OF_FOOTPRINT);
        }
        break;
        
    case MOBID_COLL_NAV_EXIT_COLLECT_AREA:
            ROS_INFO("MOBID_COLL_NAV_EXIT_COLLECT_AREA");
        if( ros::Time::now() - stamp_start_< stamp_wait_)    
            break;
        // For now just move a bit forward. Later the mobidik should exit the rail area. TODO
        tf::poseMsgToTF(base_position->pose,base_position_pose);
        robot_yaw = tf::getYaw(base_position_pose.getRotation());
        goal_.target_pose.pose = base_position->pose;
        goal_.target_pose.pose.position.x += DIST_MOVE_FRONT_POSTDOCKING*std::cos(robot_yaw);
        goal_.target_pose.pose.position.y += DIST_MOVE_FRONT_POSTDOCKING*std::sin(robot_yaw);
        
        std::cout << "goal_ = " << goal_ << std::endl;
        
        nav_next_state_wp_ = MOBID_COLL_NAV_DONE;
        nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;      
        break;

    case MOBID_COLL_NAV_GOTOPOINT:
            ROS_INFO("MOBID_COLL_NAV_GOTOPOINT");
        goal_.target_pose.header.frame_id = "map";
        goal_.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        sendgoal = true;
        tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state_ = MOBID_COLL_NAV_BUSY;
        break;
         
    case MOBID_COLL_NAV_BUSY:
            ROS_INFO("MOBID_COLL_NAV_BUSY");
        if (!isPositionValid())
        {
            nav_next_state_ = MOBID_COLL_NAV_HOLD;
            break;
        }
        if (isWaypointAchieved(xy_goal_tolerance_,yaw_goal_tolerance_)) // TODO set the tolerance parameters of the base_local_planner_params equal to the tolerances used in this function -> at the ros param server
        {
                ROS_INFO("Waypoint achieved");
            nav_next_state_ = MOBID_COLL_NAV_WAYPOINT_DONE;
        }
        else 
        {
                ROS_INFO("Waypoint not achieved");
        }

        break;

    case MOBID_COLL_NAV_WAYPOINT_DONE: //
            ROS_INFO("MOBID_COLL_NAV_DONE");
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE; // is this still valid when we integrate an extra waypoint for this mobidik-collection? What should we send as feedback?
        controlMode->data = ropodNavigation::LLC_VEL; // LLC_NORMAL
        if (nav_next_state_wp_ == MOBID_COLL_NAV_CONNECTING)
            controlMode->data = ropodNavigation::LLC_NORMAL; // for one sample send normal so the Matlab node can jump to docking
        nav_next_state_ = nav_next_state_wp_;

        break;
        
    case MOBID_COLL_NAV_DONE: //
            ROS_INFO("MOBID_COLL_NAV_DONE");
        ROS_INFO("Navigation done");
        tfb_nav.fb_nav = NAV_DONE;        
        stopNavigation();
        nav_cancel_pub.publish(true_bool_msg_);
        nav_next_state_ = MOBID_COLL_NAV_IDLE;
        break;

    case MOBID_COLL_NAV_HOLD: //
            ROS_INFO("MOBID_COLL_NAV_HOLD");
        ROS_INFO("Navigation on hold to receive feedback");
        if (isPositionValid()) // check we have a valid position
            nav_next_state_ = MOBID_COLL_NAV_BUSY;
        break;

    case MOBID_COLL_NAV_PAUSED: // this state is reached via a callback
            ROS_INFO("MOBID_COLL_NAV_PAUSED");
        if(nav_paused_req_)
        {
            nav_cancel_pub.publish(true_bool_msg_);
            nav_paused_req_ = false;
        }
        break;

    default:
        nav_next_state_ = MOBID_COLL_NAV_IDLE;
    }

    nav_state_ = nav_next_state_;
    
    mn_goal.goal = goal_.target_pose;

    return tfb_nav;
}


void MobidikCollection::getFinalMobidikPos ( const ed::WorldModel& world, std::string mobidikAreaID, geo::Pose3D *mobidikPosition, geo::Pose3D *disconnectSetpoint , geo::Pose3D *setpointInFrontOfMobidik, visualization_msgs::Marker* points )
{
    std::string orientationWPID = "orient_wp_" +  mobidikAreaID;
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it ) // find corresponding orientation
    {
        const ed::EntityConstPtr& e = *it;
        if ( orientationWPID.compare ( e.get()->id().str() ) == 0 ) // function returns 0 if strings are equal
        {
            geo::Pose3D poseWP = e.get()->pose();
            geo::Quaternion rotationWP = poseWP.getQuaternion();

            tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
            tf::Matrix3x3 matrix ( q );
            double WP_roll, WP_pitch, WP_yaw;
            matrix.getRPY ( WP_roll, WP_pitch, WP_yaw );

            geo::Mat3 rotation;
            rotation.setRPY ( WP_roll, WP_pitch, WP_yaw );

            geo::Vec3d finalPosMobidik ( poseWP.getOrigin().getX(), poseWP.getOrigin().getY(), 0.0 );
            mobidikPosition->setOrigin ( finalPosMobidik );
            mobidikPosition->setBasis ( rotation );


            geo::Vec3d originInFrontOfMobidik ( mobidikPosition->getOrigin().getX() + DIST_INTERMEDIATE_WAYPOINT_MOBID_RELEASE *cos ( WP_yaw ), mobidikPosition->getOrigin().getY() + DIST_INTERMEDIATE_WAYPOINT_MOBID_RELEASE *sin ( WP_yaw ), 0.0 );
            setpointInFrontOfMobidik->setOrigin ( originInFrontOfMobidik );
            setpointInFrontOfMobidik->setBasis ( rotation );

            geo::Vec3d originAfterDisconnection ( mobidikPosition->getOrigin().getX() + DIST_DISCONNECT_MOBID_RELEASE *cos ( WP_yaw ), mobidikPosition->getOrigin().getY() + DIST_DISCONNECT_MOBID_RELEASE *sin ( WP_yaw ), 0.0 );
            disconnectSetpoint->setOrigin ( originAfterDisconnection );
            disconnectSetpoint->setBasis ( rotation );

            geometry_msgs::Point p;
            p.x = mobidikPosition->getOrigin().getX();
            p.y = mobidikPosition->getOrigin().getY();
            p.z= 0.0;
            points->points.push_back ( p );

            p.x = setpointInFrontOfMobidik->getOrigin().getX();
            p.y = setpointInFrontOfMobidik->getOrigin().getY();
            p.z= 0.0;
            points->points.push_back ( p );

            p.x = disconnectSetpoint->getOrigin().getX();
            p.y = disconnectSetpoint->getOrigin().getY();
            p.z = 0.0;
            points->points.push_back ( p );
            return;
        }
    }
    
    return;
}

void MobidikCollection::point2goal(geo::Pose3D *setpoint)
{
        goal_.target_pose.pose.position.x = setpoint->getOrigin().getX();
        goal_.target_pose.pose.position.y = setpoint->getOrigin().getY();
        goal_.target_pose.pose.position.z = setpoint->getOrigin().getZ();
        goal_.target_pose.pose.orientation.x = setpoint->getQuaternion().getX();
        goal_.target_pose.pose.orientation.y = setpoint->getQuaternion().getY();
        goal_.target_pose.pose.orientation.z = setpoint->getQuaternion().getZ();
        goal_.target_pose.pose.orientation.w = setpoint->getQuaternion().getW();
}

TaskFeedbackCcu MobidikCollection::callReleasingStateMachine ( ros::Publisher &movbase_cancel_pub, maneuver_navigation::Goal &mn_goal, bool& sendgoal, 
                                                               visualization_msgs::MarkerArray markerArray, std::string areaID, const ed::WorldModel& world,
                                                               ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest, std_msgs::UInt16* controlMode, 
                                                               ros::Publisher &cmv_vel_pub, ropodNavigation::wrenches bumperWrenches, bool *mobidikConnected,  const bool robotReal,
                                                               ros::Publisher &dockingCommand, ropod_ros_msgs::DockingFeedback dockingFeedback)
{
    //TODO set controlmode in all the states
    TaskFeedbackCcu tfb_nav;
    tfb_nav.wayp_n = waypoint_cnt_;
    tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;
    geo::Quaternion rotationWP;
    geo::Pose3D rotationSetpoint;
    tf::Quaternion q;
    tf::Matrix3x3 matrix;
    double WP_roll, WP_pitch, WP_yaw;
    geo::Mat3 rotation;
    geometry_msgs::Twist output_vel;
    bool touched, forceCheck;
    float avgForce;
    tf::Pose goal_tfpose;
    tf::Quaternion quat_temp;
    double robot_yaw;
    double ref_vector_yaw;
    double setpoint_yaw;
    int sysCommand;
    
    /* Just for visualisation purposes TODO to be removed */
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    
     // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
// ---------------------------------------------------------------------------
    
    mn_goal.conf.precise_goal = true;
    mn_goal.conf.use_line_planner = false;
    mn_goal.conf.append_new_maneuver =  false;
    
    mn_goal.start.pose = base_position->pose; // always plan from current pose
    
    mn_goal.goal.header.frame_id = "map";
    mn_goal.goal.header.stamp = ros::Time::now();
    mn_goal.start.header.frame_id = "map";
    mn_goal.start.header.stamp = ros::Time::now();
    
    ropod_ros_msgs::DockingCommand dockingMsg;
    
    switch ( nav_state_release_ )
    {

    case MOBID_REL_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
        ROS_INFO ( "MOBID_REL_NAV_IDLE" );
        break;

    case MOBID_REL_GET_SETPOINT_FRONT:
        ROS_INFO ( "MOBID_REL_GET_SETPOINT_FRONT" );
        
                    // Configure slow movement
        sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_x 0.3 &");
        sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_theta 0.8 &");                       

        getFinalMobidikPos ( world, areaID, &finalMobidikPosition_, &disconnectSetpoint_, &setpoint_, &points );
        
        // Find pose parallel to the delvery area
        tf::poseMsgToTF(base_position->pose,goal_tfpose);
        robot_yaw = tf::getYaw(goal_tfpose.getRotation());
        quat_temp.setW(setpoint_.getQuaternion().getW());
        quat_temp.setX(setpoint_.getQuaternion().getX());
        quat_temp.setY(setpoint_.getQuaternion().getY());
        quat_temp.setZ(setpoint_.getQuaternion().getZ());
        ref_vector_yaw = tf::getYaw(quat_temp);
        
        setpoint_yaw = angles::normalize_angle(ref_vector_yaw);
        
//         if ( std::cos( (ref_vector_yaw + 0.5*M_PI) -  robot_yaw)> 0.0 )
//             setpoint_yaw = angles::normalize_angle(ref_vector_yaw + 0.5*M_PI);
//         else            
//             setpoint_yaw = angles::normalize_angle(ref_vector_yaw - 0.5*M_PI);
        
        quat_temp.setRPY(0.0,0.0,setpoint_yaw);
        
        goal_tfpose.setOrigin(tf::Vector3(setpoint_.getOrigin().getX(), setpoint_.getOrigin().getY(), setpoint_.getOrigin().getZ()));
        goal_tfpose.setRotation(quat_temp);
        tf::poseTFToMsg(goal_tfpose,goal_.target_pose.pose);  
        
        markerArraytest->markers.push_back( points );
        
        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_GOTO_SETPOINT_FRONT;
        break;

    case MOBID_REL_GOTO_SETPOINT_FRONT:
        ROS_INFO ( "MOBID_REL_GOTO_SETPOINT_FRONT" );
        sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_x_backwards 0.3 &");    
        point2goal(&setpoint_);
     
        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_GOTO_FINAL_MOBIDIK_POS;
        break;        
    case MOBID_REL_GOTO_FINAL_MOBIDIK_POS:
        ROS_INFO ( "MOBID_REL_GOTO_FINAL_MOBIDIK_POS" );
        sysCommand = system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_x_backwards 0.3 &");         
        // navigate to final setpoint
        point2goal(&finalMobidikPosition_);

        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_INIT_DECOUPLING;
        bumperWrenchesVector_.clear();
        break;
        
    case MOBID_REL_INIT_DECOUPLING:
            ROS_INFO ( "MOBID_REL_INIT_DECOUPLING" );
            dockingMsg.docking_command = DOCKING_COMMAND_RELEASE;
            dockingCommand.publish( dockingMsg );
            nav_next_state_release_ = MOBID_REL_DECOUPLING;

    case MOBID_REL_DECOUPLING:
        ROS_INFO ( "MOBID_REL_DECOUPLING" );
        // Wait untill signal is given with force sensor at the front and go to the position which is slightly in front of the robot.

        touched = false;

        if ( !robotReal )
        {
            touched = true;
            ROS_WARN ( " You are in simulation mode. The mobidik is assumed to be disconnected now." );
        }
        else
        {
//             bumperWrenchesVector_.push_back ( bumperWrenches );
//             if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES ) // TODO update goal and how? How to let the software know there is actually no goal
//             {
//                 bumperWrenchesVector_.erase ( bumperWrenchesVector_.begin() );
// 
//                 if ( !avgWrenchesDetermined_ )
//                 {
//                     determineAvgWrenches();
//                 }
//                 avgForce = 0.0;
//                 for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ )
//                 {
//                     avgForce += bumperWrenchesVector_[ii].front.wrench.force.x;
//                 }
// 
//                 avgForce /= bumperWrenchesVector_.size();
//                 forceCheck = std::fabs ( avgForce - avgWrenches_.front.wrench.force.x ) > MIN_FORCE_TOUCHED;
// 
//                 if ( forceCheck )
//                 {
//                     touched = true;
//                 }
// 
//             }
                
                if( dockingFeedback.docking_status == DOCKING_FB_REST )
                {
                        ROS_INFO("MOBIDIK RELEASED");
                        touched = true;
                }
        }
        
        
        
        if ( touched )
        {                 
            *mobidikConnected = false;
            tfb_nav.fb_nav = NAV_UNDOCKED;
            nav_next_state_release_ = MOBID_REL_NAV_WAIT_CHANGE_FOOTPRINT;
            stamp_start_ = ros::Time::now();
            stamp_wait_ = ros::Duration(TIME_WAIT_CHANGE_OF_FOOTPRINT);
            controlMode->data = ropodNavigation::LLC_VEL;
            xy_goal_tolerance_  = GOAL_MOBID_REL_REACHED_DIST; // for the last part we decrease tolerance
            yaw_goal_tolerance_ = GOAL_MOBID_REACHED_ANG_UNDOCK; 
        }
        
        break;
        
    case MOBID_REL_NAV_WAIT_CHANGE_FOOTPRINT:
        ROS_INFO ( "MOBID_REL_NAV_WAIT_CHANGE_FOOTPRINT" );
        if( ros::Time::now() - stamp_start_< stamp_wait_)
            break;
        tf::poseMsgToTF(base_position->pose,goal_tfpose);
        robot_yaw = tf::getYaw(goal_tfpose.getRotation());
        goal_.target_pose.pose = base_position->pose;
        goal_.target_pose.pose.position.x += DIST_MOVE_FRONT_POSTRELEASING*std::cos(robot_yaw);
        goal_.target_pose.pose.position.y += DIST_MOVE_FRONT_POSTRELEASING*std::sin(robot_yaw);  
        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_ROTATE;
        break;
        
    case MOBID_REL_ROTATE: // TODO should be removed: rotating now done to measure the environment again, while we actually know that we just disconnected the mobidik
        ROS_INFO ( "MOBID_REL_ROTATE" );
        tf::poseMsgToTF(base_position->pose,goal_tfpose);
        robot_yaw = tf::getYaw(goal_tfpose.getRotation());        
        quat_temp.setRPY(0.0,0.0,angles::normalize_angle(robot_yaw+M_PI));
        goal_tfpose.setRotation(quat_temp);
        tf::poseTFToMsg(goal_tfpose,goal_.target_pose.pose);  
        
        controlMode->data = ropodNavigation::LLC_VEL; //LLC_NORMAL
        
        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_NAV_DONE;
        break;

    case MOBID_REL_NAV_GOTOPOINT:
        ROS_INFO ( "MOBID_REL_NAV_GOTOPOINT" );
        goal_.target_pose.header.frame_id = "map";
        goal_.target_pose.header.stamp = ros::Time::now();
        ROS_INFO ( "Sending goal" );
        sendgoal = true;
        tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state_release_ = MOBID_REL_NAV_BUSY;
        break;

    case MOBID_REL_NAV_BUSY:
        ROS_INFO ( "MOBID_REL_NAV_BUSY" );
        if ( !isPositionValid() )
        {
            nav_next_state_release_ = MOBID_REL_NAV_HOLD;
            break;
        }
        if ( isWaypointAchieved(xy_goal_tolerance_,yaw_goal_tolerance_) ) // TODO set the tolerance parameters of the base_local_planner_params equal to the tolerances used in this function -> at the ros param server
        {
            ROS_INFO ( "Waypoint achieved" );
            nav_next_state_release_ = MOBID_REL_NAV_WAYPOINT_DONE;
        }
        else
        {
            ROS_INFO ( "Waypoint not achieved" );
        }

        break;

    case MOBID_REL_NAV_WAYPOINT_DONE: //
        ROS_INFO ( "MOBID_REL_NAV_WAYPOINT_DONE" );
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE; // is this still valid when we integrate an extra waypoint for this mobidik-collection? What should we send as feedback?
        nav_next_state_release_ = nav_next_state_wp_release_;
        break;

    case MOBID_REL_NAV_DONE: //
        ROS_INFO ( "MOBID_REL_NAV_DONE" );
        ROS_INFO ( "Navigation done" );
        tfb_nav.fb_nav = NAV_DONE;
        controlMode->data = ropodNavigation::LLC_VEL; //LLC_NORMAL
        stopNavigation();
        movbase_cancel_pub.publish ( true_bool_msg_ );
        nav_next_state_release_ = MOBID_REL_NAV_IDLE;
        break;

    case MOBID_REL_NAV_HOLD: //
        ROS_INFO ( "MOBID_REL_NAV_HOLD" );
        ROS_INFO ( "Navigation on hold to receive feedback" );
        if ( isPositionValid() ) // check we have a valid position
            nav_next_state_ = MOBID_REL_NAV_BUSY;
        break;

    case MOBID_REL_NAV_PAUSED: // this state is reached via a callback
        ROS_INFO ( "MOBID_REL_NAV_PAUSED" );
        if ( nav_paused_req_ )
        {
            movbase_cancel_pub.publish ( true_bool_msg_ );
            nav_paused_req_ = false;
        }
        break;

    default:
        nav_next_state_ = MOBID_REL_NAV_IDLE;
    }

    nav_state_release_ = nav_next_state_release_;
    mn_goal.goal = goal_.target_pose;

    return tfb_nav;
}
