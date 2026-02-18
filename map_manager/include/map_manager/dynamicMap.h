/*
	FILE: dynamicMap.h
	--------------------------------------
	header file dynamic map
*/

#ifndef MAPMANAGER_DYNAMICMAP_H
#define MAPMANAGER_DYNAMICMAP_H

#include <map_manager/occupancyMap.h>
#include <onboard_detector/dynamicDetector.h>
#include <rclcpp/rclcpp.hpp>

namespace mapManager{

	class dynamicMap : public occMap{
	private:

	protected:
		std::shared_ptr<onboardDetector::dynamicDetector> detector_;
		rclcpp::TimerBase::SharedPtr freeMapTimer_;


	public:
		dynamicMap(const std::shared_ptr<rclcpp::Node>& node, bool freeMap=true);

		
		void initMap(bool freeMap=true);

		// dynamic clean 
		void freeMapCB();
		void cleanDynamicObstacles() override;

		// user function
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclePos, 
								 std::vector<Eigen::Vector3d>& obstaclesVel, 
			                     std::vector<Eigen::Vector3d>& obstacleSize);
		std::shared_ptr<onboardDetector::dynamicDetector> getDetector();
	};

}

#endif