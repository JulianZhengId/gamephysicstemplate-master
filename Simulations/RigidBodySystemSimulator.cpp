#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();
	m_mouse = Point2D();
	m_oldtrackmouse = Point2D();
	m_trackmouse = Point2D();
	rigidBodies = {};
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo1, Demo2, Demo3, Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
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
		float inputScale = 0.01f;
		inputWorld = inputWorld * inputScale;
		applyExternalForce(inputWorld);
	}
}


void RigidBodySystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}


void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: //Demo1
		std::cout << "Demo1!\n";
		demo1Setup();
		simulateTimestep(2);
		std::cout << "position: " << rigidBodies[0].position << "\n";
		std::cout << "velocity: " << rigidBodies[0].velocity << "\n";
		std::cout << "angular velocity: " << rigidBodies[0].angularVelocity << "\n";
		std::cout << "angular momentum: " << rigidBodies[0].angularMomentum << "\n";
		break;
	case 1: //Demo2
		std::cout << "Demo2!\n";
		break;
	case 2: //Demo3
		std::cout << "Demo3!\n";
		break;
	case 3: //Demo4
		std::cout << "Demo4!\n";
		break;
	default:
		std::cout << "Empty Demo!\n";
		break;
	}
}

void RigidBodySystemSimulator::demo1Setup()
{
	rigidBodies = {};
	this->addRigidBody(Vec3(0, 0, 0), Vec3(1.0f, 0.6f, 0.5f), 2);
	this->setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
	this->applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (int i = 0; i < rigidBodies.size(); i++)
	{
		//data
		struct RigidBody* rb = &rigidBodies.at(i);

		Vec3 totalForce = m_externalForce;
		Vec3 torque = Vec3(0, 0, 0);
		for (int i = 0; i < rb->forces.size(); i++) {
			struct ForceOnBody* f = rb->forces.at(i);
			//accumulate external forces
			totalForce += f->magnitude;

			//calculate torque
			torque += cross(f->location, f->magnitude);
		}

		//center mass position
		rb->position += timeStep * rb->velocity;
		rb->velocity += timeStep * totalForce / rb->mass;

		//update quaternion
		Quat q = Quat(0, rb->angularVelocity.x, rb->angularVelocity.y, rb->angularVelocity.z) * rb->orientation;
		rb->orientation += timeStep / 2 * q;

		//update angular momentum
		rb->angularMomentum += timeStep * torque;

		//calculate inversed inertia tensor
		matrix4x4<double> rotationMatrix = rb->orientation.getRotMat();
		matrix4x4<double> inversedRotationMatrix = rotationMatrix.inverse();
		matrix4x4<double> inversedCurrentInertiaTensor = (rotationMatrix * rb->inversedInitialInertiaTensor * inversedRotationMatrix);

		//calculate angular velocity
		rb->angularVelocity = inversedCurrentInertiaTensor * rb->angularMomentum;

		//update world position (?)

		//clean forces
		m_externalForce = Vec3(0, 0, 0);
		rb->forces = {};
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (m_iTestCase == 0) return;
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	if (i >= rigidBodies.size()) return NAN;
	return rigidBodies.at(i).position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	if (i >= rigidBodies.size()) return NAN;
	return rigidBodies.at(i).velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	if (i >= rigidBodies.size()) return NAN;
	return rigidBodies.at(i).angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	if (i >= 0 && i < rigidBodies.size()) {
		struct RigidBody* rb = &rigidBodies.at(i);
		struct ForceOnBody* f = new ForceOnBody{ loc, force };
		rb->forces.push_back(f);
	}
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	float mxx = mass * (size.y * size.y + size.z * size.z) / 12.0f;
	float myy = mass * (size.x * size.x + size.z * size.z) / 12.0f;
	float mzz = mass * (size.x * size.x + size.y * size.y) / 12.0f;
	
	matrix4x4<double> initialInertiaTensor = matrix4x4<double>(mxx, 0, 0, 0, 0, myy, 0, 0, 0, 0, mzz, 0, 0, 0, 0, 1);
	rigidBodies.push_back(RigidBody{ position, size, mass, Quat(0, 0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), initialInertiaTensor.inverse() });
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	if (i >= 0 && i < rigidBodies.size()) {
		RigidBody* rb = &rigidBodies.at(i);
		rb->orientation = orientation;
	}
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	if (i >= 0 && i < rigidBodies.size()) {
		RigidBody* rb = &rigidBodies.at(i);
		rb->velocity = velocity;
	}
}
