#include "MassSpringSystemSimulator.h"

bool grav = false;
bool collCheck = false;

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	currMP = 0;
	m_iIntegrator = 0;
	m_fMass = 2;
	m_fStiffness = 4;
	m_fDamping = 0;
	m_mouse = Point2D();
	m_oldtrackmouse = Point2D();
	m_trackmouse = Point2D();

	addMassPoint(Vec3{-0.2,0.2,0}, Vec3(0, 0, 0), true);
	addMassPoint(Vec3{ 0,0.2,0 }, Vec3(0, 0, 0), true);
	addMassPoint(Vec3{ 0.2,0.2,0 }, Vec3(0, 0, 0), true);

	addMassPoint(Vec3{ -0.2,0,0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0,0,0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0.2,0,0 }, Vec3(0, 0, 0), false);

	addMassPoint(Vec3{ -0.2, -0.2,0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0, -0.2, 0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0.2, -0.2, 0 }, Vec3(0, 0, 0), false);

	addMassPoint(Vec3{ -0.2,-0.4,0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0,-0.4,0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0.2,-0.4,0 }, Vec3(0, 0, 0), false);

	//Left to right
	addSpring(0, 1, 0.2);
	addSpring(1, 2, 0.2);
	addSpring(3, 4, 0.2);
	addSpring(4, 5, 0.2);
	addSpring(6, 7, 0.2);
	addSpring(7, 8, 0.2);
	addSpring(9, 10, 0.2);
	addSpring(10, 11, 0.2);

	//Top to bottom
	addSpring(0, 3, 0.2);
	addSpring(3, 6, 0.2);
	addSpring(6, 9, 0.2);

	addSpring(1, 4, 0.2);
	addSpring(4, 7, 0.2);
	addSpring(7, 10, 0.2);

	addSpring(2, 5, 0.2);
	addSpring(5, 8, 0.2);
	addSpring(8, 11, 0.2);
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler, Leapfrog, Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &size, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &timestep, "min=0.0001 step=0.001");
	TwAddButton(DUC->g_pTweakBar, "Add Gravity", [](void* s) {grav ? grav = false : grav = true; }, nullptr, "");
	TwAddButton(DUC->g_pTweakBar, "Collision", [](void* s) {collCheck ? collCheck = false : collCheck = true; }, nullptr, "");
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iIntegrator)
	{
	case 0: 
		drawMassPoints(); 
		break;
	case 1: 
		break;
	case 2:
		drawMassPoints();
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int integratorCases)
{
	m_iIntegrator = integratorCases;
	switch (m_iIntegrator)
	{
	case 0:
		std::cout << "Euler!\n";
		break;
	case 1:
		std::cout << "Leapfrog!\n";
		break;
	case 2:
		std::cout << "Midpoint!\n";
		break;
	default:
		std::cout << "Empty!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
}

void MassSpringSystemSimulator::simulateTimestep(float)
{
	// update current setup for each frame
	switch (m_iIntegrator)
	{
	case 0:
		EulerIntegration(timestep);
		break;
	case 1:
		LeapFrogIntegration();
		break;
	case 2:
		MidPointIntegration(timestep);
		break;
	}
	

}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}



void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	massPoints.push_back(MassPoint{ position, velocity, isFixed });
	return currMP++;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	springs.push_back(Spring{ masspoint1, masspoint2, initialLength });
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return massPoints.at(index).pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return massPoints.at(index).vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}



void MassSpringSystemSimulator::EulerIntegration(float h) {
	for (Spring& spring : springs) {
		MassPoint& m1 = massPoints.at(spring.m1);
		MassPoint& m2 = massPoints.at(spring.m2);

		Vec3 d = m1.pos - m2.pos;
		float d_length = sqrt(pow(d.x, 2) + pow(d.y, 2) + pow(d.z, 2));
		d /= d_length;

		// Forces
		Vec3 x1 = -m_fStiffness * (d_length - spring.dr) * d;
		Vec3 x2 = -x1;

		// To acceleration
		x1 /= m_fMass;	
		x2 /= m_fMass;

		if (grav) {
			x1 += gravity;
			x2 += gravity;
		}

		// newPos = oldPos + timestep * oldVel
		if (!m1.isFixed)
			m1.pos += (h * m1.vel);

		if (!m2.isFixed)
			m2.pos += (h * m2.vel);


		//std::cout << "Timestep: " << h << std::endl;
		//std::cout << "Pos M1: " << m1.pos.x << std::endl;
		//std::cout << "Pos M2: " << m2.pos.x << std::endl << std::endl;

		// newVel = oldVel + timestep * acc at oldPos
		m1.vel += (h * x1);
		m2.vel += (h * x2);

		if (collCheck) {
			collisionCheck(m1);
			collisionCheck(m2);
		}
	}
}

void MassSpringSystemSimulator::MidPointIntegration(float h) {
	for (Spring& spring : springs) {
		MassPoint& m1 = massPoints.at(spring.m1);
		MassPoint& m2 = massPoints.at(spring.m2);

		Vec3 d = m1.pos - m2.pos;
		float d_length = sqrt(pow(d.x, 2) + pow(d.y, 2) + pow(d.z, 2));
		d /= d_length;

		// Forces old pos
		Vec3 x1 = -m_fStiffness * (d_length - spring.dr) * d;
		Vec3 x2 = -x1;

		// To acceleration old
		x1 /= m_fMass;
		x2 /= m_fMass;

		if (grav) {
			x1 += gravity;
			x2 += gravity;
		}

		// Pos Midstep
		Vec3 p_mid1 = m1.pos + 0.5f * h * m1.vel;
		Vec3 p_mid2 = m2.pos + 0.5f * h * m2.vel;

		// Vel Midstep
		Vec3 v_mid1 = m1.vel + 0.5f * h * x1;
		Vec3 v_mid2 = m2.vel + 0.5f * h * x2;

		d = p_mid1 - p_mid2;
		d_length = sqrt(pow(d.x, 2) + pow(d.y, 2) + pow(d.z, 2));
		d /= d_length;

		// Forces midstep pos
		x1 = -m_fStiffness * (d_length - spring.dr) * d;
		x2 = -x1;

		// To acceleration midstep
		x1 /= m_fMass;
		x2 /= m_fMass;

		if (grav) {
			x1 += gravity;
			x2 += gravity;
		}

		// newPos = oldPos + timestep * vel at midstep
		if (!m1.isFixed)
			m1.pos += (h * v_mid1);

		if (!m2.isFixed)
			m2.pos += (h * v_mid2);

		//std::cout << "Pos M1: " << m1.pos.x << std::endl;
		//std::cout << "Pos M2: " << m2.pos.x << std::endl << std::endl;

		// newVel = oldVel + timestep * acc at midstep
		m1.vel += (h * x1);
		m2.vel += (h * x2);

		if (collCheck) {
			collisionCheck(m1);
			collisionCheck(m2);
		}
	}
}

void MassSpringSystemSimulator::LeapFrogIntegration() {

}

void MassSpringSystemSimulator::collisionCheck(MassPoint& mp) {
	if (mp.pos.x > 0.5f) {
		mp.pos.x = 0.5f;
		mp.vel.x = 0;
	}
	else if (mp.pos.x < -0.5f) {
		mp.pos.x = -0.5f;
		mp.vel.x = 0;
	}

	if (mp.pos.y > 1) {
		mp.pos.y = 1;
		mp.vel.y = 0;
	}
	else if (mp.pos.y < -1) {
		mp.pos.y = -1;
		mp.vel.y = 0;
	}
}

void MassSpringSystemSimulator::drawMassPoints() {
	for (Spring& spring : springs) {
		MassPoint& m1 = massPoints.at(spring.m1);
		MassPoint& m2 = massPoints.at(spring.m2);

		DUC->beginLine();
		DUC->drawLine(m1.pos, Vec3(1, 1, 1), m2.pos, Vec3(1, 1, 1));
		DUC->endLine();

		DUC->setUpLighting(Vec3(), Vec3(0.4, 0.4, 0.4), 100, Vec3(1, 1, 1));
		DUC->drawSphere(m1.pos, Vec3(size, size, size));
		DUC->setUpLighting(Vec3(), Vec3(0.4, 0.4, 0.4), 100, Vec3(1, 1, 1));
		DUC->drawSphere(m2.pos, Vec3(size, size, size));
	}
}