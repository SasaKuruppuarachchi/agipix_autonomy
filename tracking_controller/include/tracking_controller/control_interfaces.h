/*
	FILE: control_interfaces.h
	---------------------------------
	controller backend abstraction interfaces
*/
#ifndef TRACKING_CONTROLLER_CONTROL_INTERFACES_H
#define TRACKING_CONTROLLER_CONTROL_INTERFACES_H

#include <Eigen/Dense>
#include <cstdint>

namespace controller {

struct ControllerState {
	Eigen::Vector3d position{0.0, 0.0, 0.0};
	Eigen::Vector3d velocity{0.0, 0.0, 0.0};
	Eigen::Vector4d attitudeQuat{1.0, 0.0, 0.0, 0.0};
};

struct ControllerTarget {
	Eigen::Vector3d position{0.0, 0.0, 0.0};
	Eigen::Vector3d velocity{0.0, 0.0, 0.0};
	Eigen::Vector3d acceleration{0.0, 0.0, 0.0};
	double yaw{0.0};
};

class StateProvider {
public:
	virtual ~StateProvider() = default;
	virtual bool getState(ControllerState& state) = 0;
	virtual bool getTarget(ControllerTarget& target) = 0;
};

class SetpointSink {
public:
	virtual ~SetpointSink() = default;
	virtual void publishBodyRateThrust(const Eigen::Vector4d& cmd) = 0;
	virtual void publishAttitudeThrust(const Eigen::Vector4d& attitudeQuat, double thrust) = 0;
	virtual void publishAccelerationYaw(
		const Eigen::Vector3d& accRef,
		double yaw,
		const Eigen::Vector3d& posRef,
		const Eigen::Vector3d& velRef,
		std::uint8_t typeMask) = 0;
};

}  // namespace controller

#endif
