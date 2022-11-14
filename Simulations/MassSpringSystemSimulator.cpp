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

const char* MassSpringSystemSimulator::getIntegratorStr() {
	return "Euler, Leapfrog, Midpoint";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

//--------------------------------------------------------------------------------------
// ELEMENTS
//--------------------------------------------------------------------------------------

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

void MassSpringSystemSimulator::addCube(MassPoint vertex, float edgeLength)
{	
	MassPoint cube[8] = { vertex };
	cube[1].position.x += edgeLength;
	cube[2].position.y += edgeLength;
	cube[3].position.z += edgeLength;
	cube[4].position.x += edgeLength;cube[4].position.y += edgeLength;
	cube[5].position.x += edgeLength; cube[5].position.z += edgeLength;
	cube[6].position.y += edgeLength; cube[6].position.z += edgeLength;
	cube[7].position += edgeLength;

	int N = massPoints.size() - 1;
	for (int i = 1; i < 8; i++) {
		massPoints.push_back(cube[i]);
		for (int j = 0; j < massPoints.size()-1; j++) {
			Spring newSpring;
			newSpring.massPoint1 = N+i;
			newSpring.massPoint2 = N+j;
			newSpring.initialLength = edgeLength;
			springs.push_back(newSpring);
		}
	}
}

//--------------------------------------------------------------------------------------
// INITIAL SETTING
//--------------------------------------------------------------------------------------

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", this->getIntegratorStr());

	switch (m_iTestCase)
	{
	case 0: 
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		break;
	case 1: break;
	case 2: break;
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.01 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.01 step=0.01");
		break;
	case 4: break;
	default: break;
	}
}

//--------------------------------------------------------------------------------------
// DEMO SET UP
//--------------------------------------------------------------------------------------

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

void MassSpringSystemSimulator::demo4Setup()
{
	massPoints = {};
	springs = {};
	this->setMass(10);
	this->setDampingFactor(0.0f);
	this->setStiffness(40);
	this->applyExternalForce(Vec3(0, 0, 0));

	int p2 = this->addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	this->addCube(massPoints[p2], 1.0);
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: //Demo1
		std::cout << "Demo1!\n";
		demo123Setup();
		break;
	case 1: //Demo2
		std::cout << "Demo2!\n";
		demo123Setup();
		m_iIntegrator = EULER;
		break;
	case 2: //Demo3
		std::cout << "Demo3!\n";
		demo123Setup();
		m_iIntegrator = MIDPOINT;
		break;
	case 3: //Demo4
		std::cout << "Demo4!\n";
		demo4Setup();
		m_iIntegrator = EULER;
		break;
	case 4: //Demo5
		std::cout << "Demo5!\n";
		m_iIntegrator = LEAPFROG;
		break;
	default:
		std::cout << "Empty Demo!\n";
		break;
	}
}

//--------------------------------------------------------------------------------------
// INTEGRATIONS
//--------------------------------------------------------------------------------------


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


//--------------------------------------------------------------------------------------
// DRAW SETTING
//--------------------------------------------------------------------------------------

void MassSpringSystemSimulator::drawDemo123()
{
	//DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));

	Vec3 green = Colors::Green;
	for (int i = 0; i < massPoints.size(); i++) {
		DUC->drawSphere(massPoints[i].position, Vec3(0.02, 0.02, 0.02));
	}
	for (int i = 0; i < springs.size(); i++) {
		DUC->beginLine();
		DUC->drawLine(massPoints[springs[i].massPoint1].position, green, massPoints[springs[i].massPoint2].position, green);
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::drawDemo4()
{
	Vec3 green = Colors::Green;
	for (int i = 0; i < massPoints.size(); i++) {
		DUC->drawSphere(massPoints[i].position, Vec3(0.02, 0.02, 0.02));
	}
	for (int i = 0; i < springs.size(); i++) {
		DUC->beginLine();
		DUC->drawLine(massPoints[springs[i].massPoint1].position, green, massPoints[springs[i].massPoint2].position, green);
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::drawDemo5()
{

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

void MassSpringSystemSimulator::PrintSimulation(float timeStep) {
	cout << "Print the positions and velocities after one step simulation: " << '\n';
	switch (m_iIntegrator)
	{
	case 0:
		cout << "After an Euler step:" << '\n';
		break;
	case 1:
		cout << "After a LeapFrog step:" << '\n';
		break;
	case 2:
		cout << "After a midpoint step:" << '\n';
	default:
		break;
	}
	cout << "point 0 vel (";
	cout << massPoints[0].velocity.x << ", " << massPoints[0].velocity.y << ", " << massPoints[0].velocity.z << "),  ";
	cout << "pos (";
	cout << massPoints[0].position.x << ", " << massPoints[0].position.y << ", " << massPoints[0].position.z << ")"<<'\n';
	cout << "point 1 vel (";
	cout << massPoints[1].velocity.x << ", " << massPoints[1].velocity.y << ", " << massPoints[1].velocity.z << "),  ";
	cout << "pos (";
	cout << massPoints[1].position.x << ", " << massPoints[1].position.y << ", " << massPoints[1].position.z << ")" << '\n';
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