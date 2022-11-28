#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
	m_timestep = 0.0001;
};

// Functions
const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo1, Demo2, Demo3, Demo4";
};
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &m_timestep, "step=0.0001 min=0.0001");

	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	}
};

void RigidBodySystemSimulator::reset() {};

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {};

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo1\n";
		addRigidBody(Vec3(), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat{ Vec3(0, 0, 1), 90 });
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1.0, 1.0, 0));
		simulateTimestep(2);
		std::cout << "position: " << bodies[0].pos << "\n";
		std::cout << "velocity: " << bodies[0].lin_vel << "\n";
		std::cout << "angular velocity: " << bodies[0].ang_vel << "\n";
		std::cout << "angular momentum: " << bodies[0].ang_mom << "\n";
		break;
	case 1:
		cout << "Demo2\n";
		break;
	case 2:
		cout << "Demo3\n";
		break;
	case 3:
		cout << "Demo4\n";
		break;
	}
};

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {};

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
	for (RigidBody& rb : bodies) {
		Vec3 q = Vec3();

		if (rb.extForce) {
			rb.extForce = false;

			// Claculate torque
			q = Vec3{ rb.loc.y * m_externalForce.z - rb.loc.z * m_externalForce.y,
					  rb.loc.z * m_externalForce.x - rb.loc.x * m_externalForce.z,
					  rb.loc.x * m_externalForce.y - rb.loc.y * m_externalForce.x };
		}

		// Calculate position and linear velocity
		rb.pos += timeStep * rb.lin_vel;
		rb.lin_vel += timeStep * (m_externalForce / rb.mass);

		// Calculate orientation
		Quat quat = Quat{ 0, rb.ang_vel.x, rb.ang_vel.y, rb.ang_vel.z } * rb.orient;
		rb.orient += (timeStep / 2) * quat;

		// Calculate angular momentum
		rb.ang_mom += timeStep * q;

		// Calculate inverse interia tensor
		matrix4x4<double> currInvTensor = rb.orient.getRotMat() * (rb.invInitalTensor * rb.orient.getRotMat().inverse());
		
		// Calculate angular velocity
		rb.ang_vel += currInvTensor * rb.ang_mom;

		m_externalForce = Vec3();
	}
};

void RigidBodySystemSimulator::onClick(int x, int y) {};

void RigidBodySystemSimulator::onMouse(int x, int y) {};

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies() { 
	return bodies.size(); 
};

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) { 
	return bodies.at(i).pos; 
};

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) { 
	return bodies.at(i).lin_vel; 
};

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) { 
	return bodies.at(i).ang_vel; 
};

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	RigidBody& rb = bodies.at(i);
	rb.extForce = true;
	rb.loc = loc;
	m_externalForce = force;
};

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	matrix4x4<double> initalTensor = 
		matrix4x4<double>(	mass * (pow(size.y, 2) + pow(size.z, 2)) / 12.0, 0, 0, 0,
							0, mass * (pow(size.x, 2) + pow(size.z, 2)) / 12.0, 0, 0, 
							0, 0, mass * (pow(size.x, 2) + pow(size.y, 2)) / 12.0, 0,
							0, 0, 0, 1);

	bodies.push_back(RigidBody{ position, size, Vec3(), Vec3(), Vec3(), initalTensor.inverse(), Quat(), mass});
};

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	bodies.at(i).orient = orientation;
};

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	bodies.at(i).lin_vel = velocity;
};