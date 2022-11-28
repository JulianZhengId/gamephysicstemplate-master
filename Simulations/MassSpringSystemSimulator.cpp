#include "MassSpringSystemSimulator.h"
bool collCheck = false;
float sphereSize = 0.01;

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();
	m_iIntegrator = 0;
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_mouse = Point2D();
	m_oldtrackmouse = Point2D();
	m_trackmouse = Point2D();
	massPoints = {};
	springs = {};
	gravity = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo1, Demo2, Demo3, Demo4, Demo5";
}

const char* MassSpringSystemSimulator::getIntegratorStr() {
	return "Euler, Midpoint, Leapfrog";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		applyExternalForce(inputWorld);
	}
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
	if (index >= massPoints.size()) return NAN;
	return massPoints.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	if (index >= massPoints.size()) return NAN;
	return massPoints.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int index = this->getNumberOfMassPoints();
	MassPoint newMassPoint;
	newMassPoint.position = position;
	newMassPoint.velocity = Velocity;
	newMassPoint.isFixed = isFixed;
	massPoints.push_back(newMassPoint);
	return index;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring newSpring;
	newSpring.massPoint1 = masspoint1;
	newSpring.massPoint2 = masspoint2;
	newSpring.initialLength = initialLength;
	springs.push_back(newSpring);
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", this->getIntegratorStr());
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1: break;
	case 2: break;
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &sphereSize, "min=0.01 step=0.01");
		//TwAddButton(DUC->g_pTweakBar, "Add Gravity", [](void* s) {grav ? grav = false : grav = true; }, nullptr, "");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "step=0.1 min=0");
		TwAddButton(DUC->g_pTweakBar, "Collision", [](void* s) {collCheck ? collCheck = false : collCheck = true; }, nullptr, "");
		break;
	case 4:
		break;
	default: break;
	}
}

void MassSpringSystemSimulator::demo123Setup()
{
	massPoints = {};
	springs = {};
	this->setMass(10);
	this->setDampingFactor(0.0f);
	this->setStiffness(40);
	this->applyExternalForce(Vec3(0, 0, 0));
	int p0 = this->addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = this->addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	this->addSpring(p0, p1, 1.0);
}

void MassSpringSystemSimulator::demo4Setup() {
	massPoints = {};
	springs = {};
	this->setMass(10);
	this->setDampingFactor(0.0f);
	this->setStiffness(40);
	this->applyExternalForce(Vec3(0, 0, 0));

	int p2 = this->addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	this->addMultipleMassPoints();
}

void MassSpringSystemSimulator::addMultipleMassPoints()
{
	addMassPoint(Vec3{ -0.2,0.2,0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0,0.2,0 }, Vec3(0, 0, 0), false);
	addMassPoint(Vec3{ 0.2,0.2,0 }, Vec3(0, 0, 0), false);

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
	addSpring(0, 1, 0.4);
	addSpring(1, 2, 0.4);
	addSpring(3, 4, 0.4);
	addSpring(4, 5, 0.4);
	addSpring(6, 7, 0.4);
	addSpring(7, 8, 0.4);
	addSpring(9, 10, 0.4);
	addSpring(10, 11, 0.4);

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

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: //Demo1
		std::cout << "Demo1!\n";
		demo123Setup();
		//euler
		m_iIntegrator = EULER;
		simulateTimestep(0.1);
		std::cout << "EULER\n";
		std::cout << "position m1: " << massPoints[0].position << "\n";
		std::cout << "position m2: " << massPoints[1].position << "\n";
		std::cout << "velocity m1: " << massPoints[0].velocity << "\n";
		std::cout << "velocity m2: " << massPoints[1].velocity << "\n";

		//midpoint
		m_iIntegrator = MIDPOINT;
		demo123Setup();
		simulateTimestep(0.1);
		std::cout << "MIDPOINT\n";
		std::cout << "position m1: " << massPoints[0].position << "\n";
		std::cout << "position m2: " << massPoints[1].position << "\n";
		std::cout << "velocity m1: " << massPoints[0].velocity << "\n";
		std::cout << "velocity m2: " << massPoints[1].velocity << "\n";

		break;
	case 1: //Demo2
		std::cout << "Demo2!\n";
		m_iIntegrator = EULER;
		demo123Setup();
		break;
	case 2: //Demo3
		std::cout << "Demo3!\n";
		m_iIntegrator = MIDPOINT;
		demo123Setup();
		break;
	case 3: //Demo4
		std::cout << "Demo4!\n";
		m_iIntegrator = EULER;
		demo4Setup();
		break;
	case 4: //Demo5
		std::cout << "Demo5!\n";
		m_iIntegrator = LEAPFROG;
		demo123Setup();
		break;
	default:
		std::cout << "Empty Demo!\n";
		break;
	}
}

void MassSpringSystemSimulator::collisionCheck(MassPoint& mp) {
	if (mp.position.x > 0.5f) {
		mp.position.x = 0.5f;
		mp.velocity.x = 0;
	}
	else if (mp.position.x < -0.5f) {
		mp.position.x = -0.5f;
		mp.velocity.x = 0;
	}

	if (mp.position.y > 1) {
		mp.position.y = 1;
		mp.velocity.y = 0;
	}
	else if (mp.position.y < -1) {
		mp.position.y = -1;
		mp.velocity.y = 0;
	}

	if (mp.position.z > 1) {
		mp.position.z = 1;
		mp.velocity.z = 0;
	}
	else if (mp.position.z < -1) {
		mp.position.z = -1;
		mp.velocity.z = 0;
	}
}


void MassSpringSystemSimulator::EulerIntegration(float timeStep)
{
	float h = timeStep;

	for (int i = 0; i < springs.size(); i++)
	{
		Spring* s = &springs[i];
		MassPoint* p0 = &massPoints[s->massPoint1];
		MassPoint* p1 = &massPoints[s->massPoint2];

		//get distance vector
		Vec3 d = p0->position - p1->position;
		//get distance length
		Vec3 ZERO;
		float length = sqrt(d.squaredDistanceTo(ZERO));
		//get normalized distance vector
		d /= length;

		//force
		Vec3 F0 = -1 * m_fStiffness * (length - s->initialLength) * d;
		Vec3 F1 = -1 * F0;
		F0 += m_externalForce;
		F1 += m_externalForce;

		//acceleration
		p0->acceleration = F0 / (m_fMass);
		p1->acceleration = F1 / (m_fMass);

		p0->acceleration += Vec3(0, -1 * gravity, 0);
		p1->acceleration += Vec3(0, -1 * gravity, 0);

		//position
		p0->position += (h * p0->velocity);
		p1->position += (h * p1->velocity);

		//velocity
		p0->velocity += (h * p0->acceleration);
		p1->velocity += (h * p1->acceleration);

		if (collCheck) {
			collisionCheck(*p0);
			collisionCheck(*p1);
		}

		m_externalForce = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::LeapFrogIntegration(float timeStep)
{
	float h = timeStep;

	for (int i = 0; i < springs.size(); i++)
	{
		Spring* s = &springs[i];
		MassPoint* p0 = &massPoints[s->massPoint1];
		MassPoint* p1 = &massPoints[s->massPoint2];

		//get distance vector
		Vec3 d = p0->position - p1->position;
		//get distance length
		Vec3 ZERO;
		float length = sqrt(d.squaredDistanceTo(ZERO));
		//get normalized distance vector
		d /= length;

		//force
		Vec3 F0 = -1 * m_fStiffness * (length - s->initialLength) * d;
		Vec3 F1 = -1 * F0;
		F0 += m_externalForce;
		F1 += m_externalForce;

		//acceleration
		p0->acceleration = F0 / (m_fMass);
		p1->acceleration = F1 / (m_fMass);

		p0->acceleration += Vec3(0, -1 * gravity, 0);;
		p1->acceleration += Vec3(0, -1 * gravity, 0);;

		//velocity
		p0->velocity += (h * p0->acceleration);
		p1->velocity += (h * p1->acceleration);

		//position
		p0->position += (h * p0->velocity);
		p1->position += (h * p1->velocity);

		if (collCheck) {
			collisionCheck(*p0);
			collisionCheck(*p1);
		}

		m_externalForce = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::MidPointIntegration(float timeStep)
{
	//h = 0.005
	float h = timeStep;

	for (int i = 0; i < springs.size(); i++)
	{
		struct Spring* s = &springs.at(i);
		struct MassPoint* m1 = &massPoints.at(s->massPoint1);
		struct MassPoint* m2 = &massPoints.at(s->massPoint2);
		//get distance vector
		Vec3 d = m1->position - m2->position;
		//get distance length
		float length = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
		//get normalized distance vector
		d = d / length;

		//calculate forces
		Vec3 force1 = -1 * m_fStiffness * (length - s->initialLength) * d;
		Vec3 force2 = -1 * force1;
		force1 += m_externalForce;
		force2 += m_externalForce;

		//calculate acceleration
		m1->acceleration = force1 / m_fMass;
		m2->acceleration = force2 / m_fMass;

		m1->acceleration += Vec3(0, -1 * gravity, 0);;
		m2->acceleration += Vec3(0, -1 * gravity, 0);;

		//midpoint position [midPosition = oldPosition + 0.5 * h * oldVelocity]
		Vec3 m1MidPos = m1->position + 0.5 * h * m1->velocity;
		Vec3 m2MidPos = m2->position + 0.5 * h * m2->velocity;

		//midpoint acceleration
		//get midpoint distance
		d = m1MidPos - m2MidPos;
		//get midpoint distance length
		length = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
		//get normalized midpoint distance vector
		d = d / length;

		//calculate forces
		force1 = -1 * m_fStiffness * (length - s->initialLength) * d;
		force2 = -1 * force1;
		force1 += m_externalForce;
		force2 += m_externalForce;

		//calculate midpoint acceleration
		Vec3 m1MidAccel = force1 / m_fMass;
		Vec3 m2MidAccel = force2 / m_fMass;

		m1MidAccel += Vec3(0, -1 * gravity, 0);
		m2MidAccel += Vec3(0, -1 * gravity, 0);

		//Midpoint Velocity [midVelocity = oldVelocity + 0.5 * h * oldAcceleration]
		Vec3 m1MidVelo = m1->velocity + 0.5 * h * m1->acceleration;
		Vec3 m2MidVelo = m2->velocity + 0.5 * h * m2->acceleration;

		//Actual Result
		//newPos = oldPos + h * midVelocity
		m1->position = m1->position + h * m1MidVelo;
		m2->position = m2->position + h * m2MidVelo;

		//newVel = oldVel + h * midAcceleration
		m1->velocity = m1->velocity + h * m1MidAccel;
		m2->velocity = m2->velocity + h * m2MidAccel;

		if (collCheck) {
			collisionCheck(*m1);
			collisionCheck(*m2);
		}

		m_externalForce = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iIntegrator)
	{
	case 0:
		EulerIntegration(timeStep);
		break;
	case 1:
		LeapFrogIntegration(timeStep);
		break;
	case 2:
		MidPointIntegration(timeStep);
	default:
		break;
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (m_iTestCase == 0) return;
	float size = sphereSize;
	for (Spring& spring : springs) {
		MassPoint& m1 = massPoints.at(spring.massPoint1);
		MassPoint& m2 = massPoints.at(spring.massPoint2);

		DUC->beginLine();
		DUC->drawLine(m1.position, Vec3(1, 1, 1), m2.position, Vec3(1, 1, 1));
		DUC->endLine();

		DUC->setUpLighting(Vec3(), Vec3(0.4, 0.4, 0.4), 100, Vec3(1, 1, 1));
		DUC->drawSphere(m1.position, Vec3(size, size, size));
		DUC->setUpLighting(Vec3(), Vec3(0.4, 0.4, 0.4), 100, Vec3(1, 1, 1));
		DUC->drawSphere(m2.position, Vec3(size, size, size));
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