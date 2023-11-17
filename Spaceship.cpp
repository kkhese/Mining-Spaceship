//
//  Spaceship.h
//
//  A module to represent a spaceship.
//

#include "Spaceship.h"

#include <cassert>
#include <cmath>

#include "GetGlut.h"
#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

#include "CoordinateSystem.h"
#include "Entity.h"
#include "Drone.h"

using namespace ObjLibrary;
Vector3 Doffset[5];
Vector3 Dcolor[5];

Spaceship :: Spaceship ()
		: Entity()
		, m_is_alive(false)
		, m_acceleration_main(1.0)
		, m_acceleration_manoeuver(1.0)
		, m_rotation_rate_radians(1.0)
{
	assert(!isInitialized());
	assert(invariant());
}

Spaceship :: Spaceship (const ObjLibrary::Vector3& position,
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
	assert(mass                   >  0.0);
	assert(radius                 >= 0.0);
	assert(acceleration_main      >  0.0);
	assert(acceleration_manoeuver >  0.0);
	assert(rotation_rate_radians  >  0.0);
	assert(display_list.isReady());
	assert(isInitialized());
	assert(invariant());
}

Vector3 Spaceship :: getFollowCameraPosition (double back_distance,
                                              double up_distance) const
{
	assert(isInitialized());

	CoordinateSystem camera = m_coords;
	camera.addPosition(camera.getForward() * -back_distance);
	camera.addPosition(camera.getUp()      *  up_distance);
	return camera.getPosition();
}

void Spaceship :: setupFollowCamera (double back_distance,
                                     double up_distance) const
{
	assert(isInitialized());

	CoordinateSystem camera = m_coords;
	camera.addPosition(camera.getForward() * -back_distance);
	camera.addPosition(camera.getUp() * up_distance);
	camera.setupCamera();
}

void Spaceship :: drawPath (const Entity& black_hole,
                            unsigned int point_count,
                            const ObjLibrary::Vector3& colour) const
{
	assert(isInitialized());

	Spaceship future = *this;

	glBegin(GL_LINE_STRIP);
		glColor3d(colour.x, colour.y, colour.z);
		glVertex3d(future.getPosition().x, future.getPosition().y, future.getPosition().z);

		for(unsigned int i = 1; i < point_count; i++)
		{
			double distance   = black_hole.getPosition().getDistance(getPosition());
			double delta_time = sqrt(distance) / 25.0;

			future.updatePhysics(delta_time, black_hole);

			double fraction = sqrt(1.0 - (double)(i) / point_count);
			glColor3d(colour.x * fraction, colour.y * fraction, colour.z * fraction);
			glVertex3d(future.getPosition().x, future.getPosition().y, future.getPosition().z);
		}
	glEnd();


	
}

// drawing future drones positions
void Spaceship::drawFutureD(const Entity& black_hole, const Entity& Drone, int color) const
{
	Spaceship futureD = *this;
	double pddistance;
	double arrivalt;
	double shipSpeed = futureD.getVelocity().getNorm();
	Doffset[0] = Vector3(3.0, 4.0, 0.0);
	Doffset[1] = Vector3(0.0, 8.0, -6.0);
	Doffset[2] = Vector3(0.0, 8.0, 6.0);
	Doffset[3] = Vector3(0.0, -5.0, 0.0);
	Doffset[4] = Vector3(0.0, 10.0, 0.0);
	Dcolor[0] =  Vector3(255, 128, 0);
	Dcolor[1] = Vector3(255, 0, 0);
	Dcolor[2] = Vector3(255, 255, 0);
	Dcolor[3] = Vector3(0, 0, 255);
	Dcolor[4] = Vector3(0, 255, 0);

	double delta_time = 1.0;

	Vector3 offset = m_coords.worldToLocal(futureD.getPosition()) + Doffset[color];
	Vector3 Woffset = m_coords.localToWorld(offset);
	pddistance = (Woffset).getDistance(Drone.getPosition());
	arrivalt = (sqrt((shipSpeed * shipSpeed) + (500.0 * pddistance)) - shipSpeed) * delta_time / 250.0;
	if (arrivalt > 0.0)
	{
		futureD.updatePhysics(arrivalt, black_hole);
	}

	offset = m_coords.worldToLocal(futureD.getPosition()) + Doffset[color];
	Woffset = m_coords.localToWorld(offset);
	glPushMatrix();
	glColor3ub(Dcolor[color].x, Dcolor[color].y, Dcolor[color].z);
	glTranslated(Woffset.x, Woffset.y, Woffset.z);
	glutWireOctahedron();
	glPopMatrix();
}

void Spaceship :: markDead ()
{
	m_is_alive = false;

	assert(invariant());
}

void Spaceship :: thrustMainEngine (double delta_time)
{
	assert(isInitialized());
	assert(delta_time >= 0.0);

	assert(m_coords.getForward().isUnit());
	m_velocity += m_coords.getForward() * m_acceleration_main * delta_time;

	assert(invariant());
}

void Spaceship :: thrustManoeuver (double delta_time,
                                   const Vector3& direction_world)
{
	assert(isInitialized());
	assert(delta_time >= 0.0);
	assert(direction_world.isUnit());

	m_velocity += direction_world * m_acceleration_manoeuver * delta_time;

	assert(invariant());
}

void Spaceship :: rotateAroundForward (double delta_time,
                                       bool is_backwards)
{
	assert(delta_time >= 0.0);

	double max_radians = m_rotation_rate_radians * delta_time;
	if(is_backwards)
		m_coords.rotateAroundForward(-max_radians);
	else
		m_coords.rotateAroundForward(max_radians);

	assert(invariant());
}

void Spaceship :: rotateAroundUp (double delta_time,
                                  bool is_backwards)
{
	assert(delta_time >= 0.0);

	double max_radians = m_rotation_rate_radians * delta_time;
	if(is_backwards)
		m_coords.rotateAroundUp(-max_radians);
	else
		m_coords.rotateAroundUp(max_radians);

	assert(invariant());
}

void Spaceship :: rotateAroundRight (double delta_time,
                                     bool is_backwards)
{
	assert(delta_time >= 0.0);

	double max_radians = m_rotation_rate_radians * delta_time;
	if(is_backwards)
		m_coords.rotateAroundRight(-max_radians);
	else
		m_coords.rotateAroundRight(max_radians);

	assert(invariant());
}

bool Spaceship :: invariant () const
{
	if(m_acceleration_main      <= 0.0) return false;
	if(m_acceleration_manoeuver <= 0.0) return false;
	if(m_rotation_rate_radians  <= 0.0) return false;
	return true;
}

// drawing current position of chasing drone with white torus
void Spaceship::drawDroneschase(ObjLibrary::Vector3 ShipLo, int chase)
{
	Doffset[0] = Vector3(3.0, 4.0, 0.0);
	Doffset[1] = Vector3(0.0, 8.0, -6.0);
	Doffset[2] = Vector3(0.0, 8.0, 6.0);
	Doffset[3] = Vector3(0.0, -5.0, 0.0);
	Doffset[4] = Vector3(0.0, 10.0, 0.0);
	glPushMatrix();
		m_coords.applyDrawTransformations();
		glColor3ub(255, 255, 255);
		glTranslated(Doffset[chase].x, Doffset[chase].y, Doffset[chase].z);
		glRotated(90.0, 0.0, 1.0, 0.0);
		glutWireTorus(0.5, 1.4, 10, 8);
	glPopMatrix();
}

// Drawing current drone position with withe toruses if drone is alive
void Spaceship::drawDrones(ObjLibrary::Vector3 ShipLo, int live)
{
	Vector3 Doffset1(3.0, 4.0, 0.0);
	Vector3 Doffset2(0.0, 8.0, -6.0);
	Vector3 Doffset3(0.0, 8.0, 6.0);
	Vector3 Doffset4(0.0, -5.0, 0.0);
	Vector3 Doffset5(0.0, 10.0, 0.0);

	glPushMatrix();
	m_coords.applyDrawTransformations();
	glColor3ub(255, 128, 0);
	glTranslated(Doffset1.x, Doffset1.y, Doffset1.z);
	glRotated(90.0, 0.0, 1.0, 0.0);
	if (live == 0)
	{
		glutWireTorus(0.5, 1.4, 10, 8);
	}

	glColor3ub(255, 0, 0);
	glRotated(-90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset2.x - Doffset1.x, Doffset2.y - Doffset1.y, Doffset2.z - Doffset1.z);
	glRotated(-90.0, 0.0, 1.0, 0.0);
	if (live == 1)
	{
		glutWireTorus(0.5, 1.4, 10, 8);
	}

	glColor3ub(255, 255, 0);
	glRotated(90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset3.x - Doffset2.x, Doffset3.y - Doffset2.y, Doffset3.z - Doffset2.z);
	glRotated(90.0, 0.0, 1.0, 0.0);
	if (live == 2)
	{
		glutWireTorus(0.5, 1.4, 10, 8);
	}

	glColor3ub(0, 0, 255);
	glRotated(-90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset4.x - Doffset3.x, Doffset4.y - Doffset3.y, Doffset4.z - Doffset3.z);
	glRotated(-90.0, 0.0, 1.0, 0.0);
	if (live == 3)
	{
		glutWireTorus(0.5, 1.4, 10, 8);
	}

	glColor3ub(0, 255, 0);
	glRotated(90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset5.x - Doffset4.x, Doffset5.y - Doffset4.y, Doffset5.z - Doffset4.z);
	glRotated(90.0, 0.0, 1.0, 0.0);
	if (live == 4)
	{
		glutWireTorus(0.5, 1.4, 10, 8);
	}
	glPopMatrix();
}

// Drawing dead drones with black toruses
void Spaceship::drawDronesdead(ObjLibrary::Vector3 ShipLo)
{
	Vector3 Doffset1(3.0, 4.0, 0.0);
	Vector3 Doffset2(0.0, 8.0, -6.0);
	Vector3 Doffset3(0.0, 8.0, 6.0);
	Vector3 Doffset4(0.0, -5.0, 0.0);
	Vector3 Doffset5(0.0, 10.0, 0.0);

	glPushMatrix();
	m_coords.applyDrawTransformations();
	glColor3ub(0, 0, 0);
	glTranslated(Doffset1.x, Doffset1.y, Doffset1.z);
	glRotated(90.0, 0.0, 1.0, 0.0);
	glutWireTorus(0.5, 1.4, 10, 8);

	glRotated(-90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset2.x - Doffset1.x, Doffset2.y - Doffset1.y, Doffset2.z - Doffset1.z);
	glRotated(-90.0, 0.0, 1.0, 0.0);
	glutWireTorus(0.5, 1.4, 10, 8);

	glRotated(90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset3.x - Doffset2.x, Doffset3.y - Doffset2.y, Doffset3.z - Doffset2.z);
	glRotated(90.0, 0.0, 1.0, 0.0);
	glutWireTorus(0.5, 1.4, 10, 8);

	glRotated(-90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset4.x - Doffset3.x, Doffset4.y - Doffset3.y, Doffset4.z - Doffset3.z);
	glRotated(-90.0, 0.0, 1.0, 0.0);
	glutWireTorus(0.5, 1.4, 10, 8);

	glRotated(90.0, 0.0, 1.0, 0.0);
	glTranslated(Doffset5.x - Doffset4.x, Doffset5.y - Doffset4.y, Doffset5.z - Doffset4.z);
	glRotated(90.0, 0.0, 1.0, 0.0);
	glutWireTorus(0.5, 1.4, 10, 8);
	glPopMatrix();
}

// Sendding escort coordinate to drones
Vector3 Spaceship::SendPosition(int droneN)
{
	Doffset[0] = Vector3(3.0, 4.0, 0.0);
	Doffset[1] = Vector3(0.0, 8.0, -6.0);
	Doffset[2] = Vector3(0.0, 8.0, 6.0);
	Doffset[3] = Vector3(0.0, -5.0, 0.0);
	Doffset[4] = Vector3(0.0, 10.0, 0.0);

	Spaceship current = *this;
	Vector3 Local = m_coords.worldToLocal(current.getPosition());
	Vector3 offset = Local + Doffset[droneN];
	Vector3 dronePosition = m_coords.localToWorld(offset);
	return dronePosition;
}
