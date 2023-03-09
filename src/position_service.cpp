/**
* \file position_service.cpp
* \brief This file implements a position service
* \author Suraj Deshini
* \version 0.1
* \date 09/03/2023
*
* \details
*
* Services : <BR>
*              /position_server
*
* Description :
*
* This node advertises a position service. When the service is required, a request containing
* minimum and maximum values for the x and y position is used to generate a random position 
* between x (or y) min and x (or y) max.
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"


/**
 * \brief random number generator
 * \param M defines the minimum possible value of the random number 
 * \param N defines the maximum possible value of the random number
  
 * \return the random number
 * 
 * This function generates a random number between M and N 
 */


double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 * \brief Service callback to define the robot target pose
 * 			  
 * \param req (const std::shared_ptr<RandomPosition::Request>):
 *   Service request, containing the x and y ranges.
 * \param res (const std::shared_ptr<RandomPosition::Response>):
 *   Service response, containing the robot pose. Including the "x" and "y" position
 *   bounded between x_min (or y_min) and x_max (or y_max) and its orientation
 *   "theta" bounded between -3.14 and 3-14
 *
 * 
*/

bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 * \brief Service Server definition to generate a random position
 *
 * This server provides the  '/position_server'
 * service via the node random_position_server.
 *******************************************
*/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
