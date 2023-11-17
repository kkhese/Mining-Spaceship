//
//  Crystal.cpp
//

#include "Crystal.h"

#include <cassert>
#include <algorithm>  // for min/max

#include "GetGlut.h"
#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

#include "CoordinateSystem.h"
#include "Entity.h"

using namespace ObjLibrary;
namespace
{
	const double RADIUS = 2.0;
	const double MASS   = 1.0;
	const double ROTATION_RATE_MAX = 6.0;  // radians / second

	double random01 ()
	{
		return rand() / (RAND_MAX + 1.0);
	}
	Vector3 Dcolor[5];
}  // end of anonymous namespace

Crystal :: Crystal ()
		: Entity()
		, m_rotation_axis(Vector3(1.0, 0.0, 0.0))
		, m_rotation_rate(0.0)
		, m_is_gone(false)
{
	assert(!isInitialized());
	assert(invariant());
}

Crystal :: Crystal (const ObjLibrary::Vector3& position,
                    const ObjLibrary::Vector3& velocity,
                    const ObjLibrary::DisplayList& display_list)
		: Entity(position,
		         velocity,
		         MASS,
		         RADIUS,
		         display_list,
		         3.0*RADIUS / 0.7)
		, m_rotation_axis(Vector3::getRandomUnitVector())
		, m_rotation_rate(std::min(random01(), random01()) * ROTATION_RATE_MAX)  // mostly rotate slowly
		, m_is_gone(false)
{
	assert(display_list.isReady());

	assert(isInitialized());
	assert(invariant());
}

void Crystal :: markGone ()
{
	assert(isInitialized());

	m_is_gone++;

	assert(invariant());
}

void Crystal :: updatePhysics (double delta_time,
                               const Entity& black_hole)
{
	assert(isInitialized());
	assert(delta_time > 0.0);

	Entity::updatePhysics(delta_time, black_hole);

	double rotation_radians = m_rotation_rate * delta_time;
	m_coords.rotateAroundArbitrary(m_rotation_axis, rotation_radians);

	assert(invariant());
}

bool Crystal :: invariant () const
{
	if(!m_rotation_axis.isUnit()) return false;
	if(m_rotation_rate < 0.0) return false;
	return true;
}

//====================================Added Fuction=======================================
//====================================Added Fuction=======================================
//====================================Added Fuction=======================================


// Drawing Crystal's future postions when they are being chased by drones
// Color matches with drones' colours
void Crystal::drawFutureD(const Entity& black_hole, const Entity& Drone, int color) const
{
	Crystal futureD = *this;
	// distance between Player and drone
	double pddistance;
	// ETA
	double arrivalt;
	double shipSpeed = futureD.getVelocity().getNorm();
	// Never used
	double delta_time = 1.0;

	Dcolor[0] = Vector3(255, 128, 0);
	Dcolor[1] = Vector3(255, 0, 0);
	Dcolor[2] = Vector3(255, 255, 0);
	Dcolor[3] = Vector3(0, 0, 255);
	Dcolor[4] = Vector3(0, 255, 0);

	Vector3 Woffset = futureD.getPosition();

	pddistance = (Woffset).getDistance(Drone.getPosition());
	arrivalt = (sqrt((shipSpeed * shipSpeed) + (500.0 * pddistance)) - shipSpeed) * delta_time / 250.0;
		
	if (arrivalt > 0.0)
	{
		futureD.updatePhysics(arrivalt, black_hole);
	}

	Woffset = futureD.getPosition();
	// Draw Future position with octohedron
	glPushMatrix();
		glColor3ub(Dcolor[color].x, Dcolor[color].y, Dcolor[color].z);
		glTranslated(Woffset.x, Woffset.y, Woffset.z);
		glScalef(8.0, 8.0, 8.0);
		glutWireOctahedron();
	glPopMatrix();
}