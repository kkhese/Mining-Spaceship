//
//  Spaceship.h
//
//  A module to represent a spaceship.
//

#include "Drone.h"

#include <cassert>
#include <cmath>

#include "GetGlut.h"
#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

#include "CoordinateSystem.h"
#include "Entity.h"

using namespace ObjLibrary;

Drone::Drone()
	: Entity()
	, m_is_alive(false)
	, m_acceleration_main(1.0)
	, m_acceleration_manoeuver(1.0)
	, m_rotation_rate_radians(1.0)
{
	assert(!isInitialized());
	assert(invariant());
}

Drone::Drone(const ObjLibrary::Vector3& position,
	const ObjLibrary::Vector3& velocity,
	double mass,
	double radius,
	double acceleration_main,
	double acceleration_manoeuver,
	double rotation_rate_radians,
	const ObjLibrary::DisplayList& display_list)
	: Entity(position, velocity, mass, radius, display_list, radius)
	, m_is_alive(true)
	, m_acceleration_main(acceleration_main)
	, m_acceleration_manoeuver(acceleration_manoeuver)
	, m_rotation_rate_radians(rotation_rate_radians)
{
	assert(mass > 0.0);
	assert(radius >= 0.0);
	assert(acceleration_main > 0.0);
	assert(acceleration_manoeuver > 0.0);
	assert(rotation_rate_radians > 0.0);
	assert(display_list.isReady());

	assert(isInitialized());
	assert(invariant());
}

Vector3 Drone::getFollowCameraPosition(double back_distance,
	double up_distance) const
{
	assert(isInitialized());

	CoordinateSystem camera = m_coords;
	camera.addPosition(camera.getForward() * -back_distance);
	camera.addPosition(camera.getUp() * up_distance);
	return camera.getPosition();
}

void Drone::setupFollowCamera(double back_distance,
	double up_distance) const
{
	assert(isInitialized());

	CoordinateSystem camera = m_coords;
	camera.addPosition(camera.getForward() * -back_distance);
	camera.addPosition(camera.getUp() * up_distance);
	camera.setupCamera();
}

void Drone::drawPath(const Entity& black_hole,
	unsigned int point_count,
	const ObjLibrary::Vector3& colour) const
{
	assert(isInitialized());

	Drone future = *this;

	glBegin(GL_POINTS);
	glColor3d(colour.x, colour.y, colour.z);
	glVertex3d(future.getPosition().x, future.getPosition().y, future.getPosition().z);

	for (unsigned int i = 1; i < point_count; i++)
	{
		double distance = black_hole.getPosition().getDistance(getPosition());
		double delta_time = sqrt(distance) / 256.0;

		future.updatePhysics(delta_time, black_hole);

		double fraction = sqrt(1.0 - (double)(i) / point_count);
		glColor3d(colour.x * fraction, colour.y * fraction, colour.z * fraction);
		glVertex3d(future.getPosition().x, future.getPosition().y, future.getPosition().z);
	}
	glEnd();
}

void Drone::markDead()
{
	m_is_alive = false;

	assert(invariant());
}

void Drone::thrustMainEngine(double delta_time)
{
	assert(isInitialized());
	assert(delta_time >= 0.0);

	assert(m_coords.getForward().isUnit());
	m_velocity += m_coords.getForward() * m_acceleration_main * delta_time;

	assert(invariant());
}

void Drone::thrustManoeuver(double delta_time,
	const Vector3& direction_world)
{
	assert(isInitialized());
	assert(delta_time >= 0.0);
	assert(direction_world.isUnit());

	m_velocity += direction_world * m_acceleration_manoeuver * delta_time;

	assert(invariant());
}

void Drone::rotateAroundForward(double delta_time,
	bool is_backwards)
{
	assert(delta_time >= 0.0);

	double max_radians = m_rotation_rate_radians * delta_time;
	if (is_backwards)
		m_coords.rotateAroundForward(-max_radians);
	else
		m_coords.rotateAroundForward(max_radians);

	assert(invariant());
}

void Drone::rotateAroundUp(double delta_time,
	bool is_backwards)
{
	assert(delta_time >= 0.0);

	double max_radians = m_rotation_rate_radians * delta_time;
	if (is_backwards)
		m_coords.rotateAroundUp(-max_radians);
	else
		m_coords.rotateAroundUp(max_radians);

	assert(invariant());
}

void Drone::rotateAroundRight(double delta_time,
	bool is_backwards)
{
	assert(delta_time >= 0.0);

	double max_radians = m_rotation_rate_radians * delta_time;
	if (is_backwards)
		m_coords.rotateAroundRight(-max_radians);
	else
		m_coords.rotateAroundRight(max_radians);

	assert(invariant());
}

bool Drone::invariant() const
{
	if (m_acceleration_main <= 0.0) return false;
	if (m_acceleration_manoeuver <= 0.0) return false;
	if (m_rotation_rate_radians <= 0.0) return false;
	return true;
}

// Player to drones offset distances are not uniform integers, 
// so too many random rotation cause offsets from designated positions.

void Drone::escort(Entity& ship, Entity& drone, Vector3 direction, double deltatime)
{
//	Drone dronehere = *this; => Using this, somehow inaccurate

	Vector3 Direction = direction - drone.getPosition();
	// To be relistic: drones jiggling a little at the current position
	Vector3 Noise ( (rand() / (RAND_MAX))*0.1 - 0.05, (rand() / (RAND_MAX)) * 0.1 - 0.05, (rand() / (RAND_MAX)) * 0.1 - 0.05);
	long double distance = direction.getDistance(drone.getPosition())/1000.0;
	Vector3 UnitDir = (Direction+Noise).getNormalizedSafe();
	
	double shipSpeed = (ship.getVelocity()).getNorm();

	double arrivalt = (sqrt((shipSpeed) * (shipSpeed)+500.0 * distance) - shipSpeed)*(deltatime) / (250.0);

	double safeSpeed = shipSpeed + 250.0 * arrivalt;
	double safeSpeed2 = shipSpeed + 25.0 * arrivalt;  //*distance * arrivalt;
	
	// If distance is more than 250, use main engine
	if (distance >= 250.0)
	{   
		m_coords.rotateToVector(UnitDir, 1.0);
		if (drone.getVelocity().getNorm() <= safeSpeed)
		{
			thrustMainEngine(deltatime);
		}
		else if (drone.getVelocity().getNorm() > safeSpeed)
		{
			m_velocity += m_coords.getForward() * (-m_acceleration_main) * deltatime;
		}
	}
	// If distance is less than 250, use 2nd engine
	else if (distance <= 250.0)
	{
		if (drone.getVelocity().getNorm() < safeSpeed2)
		{
			thrustManoeuver(deltatime, UnitDir);
		}
		else if (drone.getVelocity().getNorm() >= safeSpeed2)
		{
			m_velocity = Direction + ship.getVelocity();
		
		}
	}
}

void Drone::swallow(Entity& crystal, Entity& drone, Vector3 direction, double deltatime)
{
	Vector3 Direction = direction - drone.getPosition();
	long double distance = direction.getDistance(drone.getPosition())/1000.0;
	Vector3 UnitDir = Direction.getNormalizedSafe();

	double KSpeed = crystal.getVelocity().getNorm();

	double arrivalt = (sqrt((KSpeed) * (KSpeed)+500.0 * distance) - KSpeed)*deltatime / 250.0;

	double safeSpeed = KSpeed + 250.0 * arrivalt;

	// Use main engine if distance to crystal is more than 500
	if (distance >= 500.0)
	{
		m_coords.rotateToVector(UnitDir, 1.0);
		if (m_velocity.getNorm() <= safeSpeed)
		{
			thrustMainEngine(deltatime);
		}
		else if (m_velocity.getNorm() > safeSpeed)
		{
			m_velocity += m_coords.getForward() * (-m_acceleration_main) * deltatime;
		}
	}
	// Use 2nd engine if distance to crystal i less than 500
	else if (distance <= 500.0)
	{
		if (m_velocity.getNorm() < safeSpeed && distance > 0.0)
		{
			thrustManoeuver(deltatime, UnitDir);

		}
		else if (m_velocity.getNorm() >= safeSpeed)
		{
			m_velocity = Direction + crystal.getVelocity();
		}
	}

}
// Avoid action for drone
void Drone::avoid(Entity& ast, Entity& drone, double deltatime)
{
	Vector3 Direction = drone.getPosition() - ast.getPosition();
	Vector3 UnitDir = Direction.getNormalizedSafe();

	m_coords.rotateToVector(UnitDir, 1.0);
	thrustMainEngine(deltatime);
}

