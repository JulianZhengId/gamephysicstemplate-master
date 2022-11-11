#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iIntegrator = 0;
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_mouse = Point2D();
	m_oldtrackmouse = Point2D();
	m_trackmouse = Point2D();
	massPoints = {};
	springs = {};
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler, Leapfrog, Midpoint";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iIntegrator)
	{
	case 0:break;
		//case 1:
		//	TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//	TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		//	break;
	case 2:break;
	default:break;
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
		std::cout << "Empty Method!\n";
		break;
	}
	//m_iTestCase = testCase;
	//switch (m_iTestCase)
	//{
	//case 0:
	//	cout << "Teapot !\n";
	//	m_vfMovableObjectPos = Vec3(0, 0, 0);
	//	m_vfRotate = Vec3(0, 0, 0);
	//	break;
	//case 1:
	//	cout << "Random Object!\n";
	//	m_iNumSpheres = 100;
	//	m_fSphereSize = 0.05f;
	//	break;
	//case 2:
	//	cout << "Triangle !\n";
	//	break;
	//default:
	//	cout << "Empty Test!\n";
	//	break;
	//}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		//	Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		//	worldViewInv = worldViewInv.inverse();
		//	Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		//	Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		//	// find a proper scale!
		//	float inputScale = 0.001f;
		//	inputWorld = inputWorld * inputScale;
		//	m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	//else {
	//	m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	//}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// handling different cases
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
	// update current setup for each frame
	//switch (m_iTestCase)
	//{// handling different cases
	//case 0:
	//	// rotate the teapot
	//	m_vfRotate.x += timeStep;
	//	if (m_vfRotate.x > 2 * M_PI) m_vfRotate.x -= 2.0f * (float)M_PI;
	//	m_vfRotate.y += timeStep;
	//	if (m_vfRotate.y > 2 * M_PI) m_vfRotate.y -= 2.0f * (float)M_PI;
	//	m_vfRotate.z += timeStep;
	//	if (m_vfRotate.z > 2 * M_PI) m_vfRotate.z -= 2.0f * (float)M_PI;

	//	break;
	//default:
	//	break;
	//}
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

		//acceleration
		p0->acceleration = F0 / (m_fMass);
		p1->acceleration = F1 / (m_fMass);

		//position
		p0->position += (h * p0->velocity);
		p1->position += (h * p1->velocity);

		//velocity
		p0->velocity += (h * p0->acceleration);
		p1->velocity += (h * p1->acceleration);
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

		//acceleration
		p0->acceleration = F0 / (m_fMass);
		p1->acceleration = F1 / (m_fMass);

		//velocity
		p0->velocity += (h * p0->acceleration);
		p1->velocity += (h * p1->acceleration);

		//position
		p0->position += (h * p0->velocity);
		p1->position += (h * p1->velocity);
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

		//calculate acceleration
		m1->acceleration = force1 / m_fMass;
		m2->acceleration = force2 / m_fMass;

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

		//calculate midpoint acceleration
		Vec3 m1MidAccel = force1 / m_fMass;
		Vec3 m2MidAccel = force2 / m_fMass;

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
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iIntegrator)
	{
	case 0: EulerIntegration(0); break;
	case 1: LeapFrogIntegration(0); break;
	case 2: MidPointIntegration(0); break;
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