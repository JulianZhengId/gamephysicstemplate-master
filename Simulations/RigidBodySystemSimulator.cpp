#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();
	m_mouse = Point2D();
	m_oldtrackmouse = Point2D();
	m_trackmouse = Point2D();
	gravity = 0;
	rigidBodies = {};
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo1, Demo2, Demo3, Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: break; //demo1
	case 1: break; //demo2
	case 2: break; //demo3
	case 3: //demo4
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "step=0.005 min=0");
		break;
	case 4:
		break;
	default: break;
	}
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

		float inputScale = 0.0015f;
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
		std::cout << "angular momentum: " << rigidBodies[0].angularMomentum << "\n\n";
		break;
	case 1: //Demo2
		std::cout << "Demo2!\n";
		demo1Setup();
		break;
	case 2: //Demo3
		std::cout << "Demo3!\n";
		demo3Setup();
		break;
	case 3: //Demo4
		std::cout << "Demo4!\n";
		demo4Setup();
		break;
	default:
		std::cout << "Empty Demo!\n";
		break;
	}
}

void RigidBodySystemSimulator::demo1Setup()
{
	rigidBodies = {};
	gravity = 0;
	this->addRigidBody(Vec3(0, 0, 0), Vec3(1.0f, 0.6f, 0.5f), 2);
	this->setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
	this->applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
}

void RigidBodySystemSimulator::demo3Setup() 
{
	rigidBodies = {};
	gravity = 0;
	//Box 0 - Bottom
	this->addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 5);
	this->setOrientationOf(0, Quat(Vec3(0.0f, 1.0f, 0.0f), (float)(M_PI) * 0.25f));
	this->applyForceOnBody(0, Vec3(0.0f, 0.0f, 0.1f), Vec3(0.0f, 5.0f, 0.0f));

	//Box 1 - Top
	this->addRigidBody(Vec3(0.0f, 0.4f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 7);
	this->setOrientationOf(1, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.25f));
	this->setVelocityOf(1, Vec3(0.0f, -0.05f, 0.0f));
}

void RigidBodySystemSimulator::demo4Setup() 
{
	rigidBodies = {};
	gravity = 0;
	//Ground Floor
	this->addRigidBody(Vec3(0.0, -0.75f, 0.05), Vec3(10.0f, 0.2f, 10.0f), std::numeric_limits<double>::max());
	this->setIsFixed(0, true);
		
	//Box 0 - Cetnter
	this->addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 5);
	this->setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
	this->setVelocityOf(1, Vec3(0.0f, -0.01f, 0.025f));

	//Box 1 - Behind
	this->addRigidBody(Vec3(0.0f, 0.25f, 0.5f), Vec3(0.2f, 0.2f, 0.2f), 3);
	this->setOrientationOf(2, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.1f));
	this->applyForceOnBody(2, Vec3(0.0f, 0.0f, 0.6f), Vec3(0.0f, 5.0f, -20.0f));

	//Box 2 - Left
	this->addRigidBody(Vec3(-0.6f, 0.0f, 0.0f), Vec3(0.1f, 0.6f, 0.6f), 7);
	this->setOrientationOf(3, Quat(Vec3(1.0f, 0.0f, 1.0f), (float)(M_PI) * 0.15f));
	this->setVelocityOf(3, Vec3(0.03f, 0.0f, 0.0f));

	//Box 3 - Top
	this->addRigidBody(Vec3(0.1f, 0.7f, 0.05f), Vec3(0.1f, 0.4f, 0.1f), 5);
	this->setOrientationOf(4, Quat(Vec3(0.0f, 1.0f, 1.0f), (float)(M_PI) * 0.15f));
	this->applyForceOnBody(4, Vec3(-0.05f, 0.2f, 0.0f), Vec3(0.0f, -10.0f, 0.0f));
}


void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (int i = 0; i < rigidBodies.size(); i++)
	{
		//data
		struct RigidBody* rb = &rigidBodies.at(i);

		Vec3 totalForce = m_externalForce;
		if (!rb->isFixed) {
			totalForce += rb->mass * Vec3(0.0f, -1.0f * gravity, 0.0f);
		}

		Vec3 torque = Vec3(0, 0, 0);
		for (int j = 0; j < rb->forces.size(); j++) {
			struct ForceOnBody* f = rb->forces.at(j);
			//accumulate forces
			totalForce += f->magnitude;

			//calculate torque
			torque += cross(f->location, f->magnitude);
		}

		//center mass position and velocity
		rb->position += timeStep * rb->velocity;
		rb->velocity += timeStep * totalForce / rb->mass;

		//update quaternion
		rb->orientation += timeStep / 2.0f * Quat(rb->angularVelocity.x, rb->angularVelocity.y, rb->angularVelocity.z, 0) * rb->orientation;
		rb->orientation.norm();

		//update angular momentum
		rb->angularMomentum += timeStep * torque;

		//calculate inversed inertia tensor
		Mat4 rotationMatrix = rb->orientation.getRotMat();
		Mat4 inversedRotationMatrix = rotationMatrix.inverse();
		Mat4 inversedCurrentInertiaTensor = (rotationMatrix * rb->inversedInitialInertiaTensor * inversedRotationMatrix);

		//calculate angular velocity
		rb->angularVelocity = inversedCurrentInertiaTensor * rb->angularMomentum;

		//check collision
		checkCollision(rb);

		//clean forces
		rb->forces = {};
	}
	m_externalForce = Vec3(0, 0, 0);
}

void RigidBodySystemSimulator::checkCollision(RigidBody* rb) {
	for (int k = 0; k < rigidBodies.size(); k++) {
		struct RigidBody* otherRb = &rigidBodies.at(k);

		if (rb == otherRb) continue;

		collisionResponse(rb, otherRb);
	}
}

void RigidBodySystemSimulator::setIsFixed(int i, bool isFixed)
{
	if (i >= 0 && i < rigidBodies.size()) {
		RigidBody* rb = &rigidBodies.at(i);
		rb->isFixed = isFixed;
	}
}

void RigidBodySystemSimulator::collisionResponse(RigidBody* A, RigidBody* B) {
	//first detect if collision is valid
	//if so, calculate impulse, and update velocity / angular momentum

	Mat4 obj2World_A = TransferMatrix(A);
	Mat4 obj2World_B = TransferMatrix(B);

	CollisionInfo collisionCheck = checkCollisionSAT(obj2World_A, obj2World_B);

	//when a collision occurs
	if (collisionCheck.isValid) {
		Vec3 normal = collisionCheck.normalWorld;

		Vec3 collisionRelPositionA = collisionCheck.collisionPointWorld - A->position;
		Vec3 collisionRelPositionB = collisionCheck.collisionPointWorld - B->position;
		Vec3 vi_A = A->velocity + cross(A->angularVelocity, collisionRelPositionA);
		Vec3 vi_B = B->velocity + cross(B->angularVelocity, collisionRelPositionB);
		Vec3 v_Rel = vi_A - vi_B;
		
		//apply impulse when both bodies are not separating
		float separatingValue = dot(v_Rel, normal);
		if (separatingValue < 0) {
			//A current inversed inertia tensor
			Mat4 rotationMatrixA = A->orientation.getRotMat();
			Mat4 inversedRotationMatrixA = rotationMatrixA.inverse();
			Mat4 inversedCurrentInertiaTensorA = (rotationMatrixA * A->inversedInitialInertiaTensor * inversedRotationMatrixA);

			//B current inversed inertia tensor
			Mat4 rotationMatrixB = B->orientation.getRotMat();
			Mat4 inversedRotationMatrixB = rotationMatrixB.inverse();
			Mat4 inversedCurrentInertiaTensorB = (rotationMatrixB * B->inversedInitialInertiaTensor * inversedRotationMatrixB);

			//Impulse
			Vec3 impulse = -1.0f * (1 + bounciness) * separatingValue;
			Vec3 crossA = cross(inversedCurrentInertiaTensorA * cross(collisionRelPositionA, normal), collisionRelPositionA);
			Vec3 crossB = cross(inversedCurrentInertiaTensorB * cross(collisionRelPositionB, normal), collisionRelPositionB);
			float divisor = (1.0f / A->mass + 1.0f / B->mass + dot((crossA + crossB), normal));
			impulse /= divisor;

			//update velocity and angular momentum
			A->velocity += (impulse * normal) / A->mass;
			B->velocity -= (impulse * normal) / B->mass;

			Vec3 a = impulse * normal;
			A->angularMomentum += cross(collisionRelPositionA, (impulse * normal));
			B->angularMomentum -= cross(collisionRelPositionB, (impulse * normal));
		}
	}
}

Mat4 RigidBodySystemSimulator::TransferMatrix(RigidBody* rb) {
	//set Matrix
	Mat4 scaleMat, rotMat, translationMat;

	scaleMat.initScaling(rb->size.x, rb->size.y, rb->size.z);
	rotMat = rb->orientation.getRotMat();
	translationMat.initTranslation(rb->position.x, rb->position.y, rb->position.z);

	return scaleMat * rotMat * translationMat;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (m_iTestCase == 0) return;
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));


	for (RigidBody& rb : rigidBodies) {
		//set Matrix
		Mat4 scaleMat, rotMat, translationMat;

		scaleMat.initScaling(rb.size.x, rb.size.y, rb.size.z);
		
		rotMat = rb.orientation.getRotMat();
		translationMat.initTranslation(rb.position.x, rb.position.y, rb.position.z);

		DUC->drawRigidBody(scaleMat * rotMat * translationMat);
	}
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
	
	Mat4 initialInertiaTensor = Mat4(mxx, 0, 0, 0, 0, myy, 0, 0, 0, 0, mzz, 0, 0, 0, 0, 1);
	rigidBodies.push_back(RigidBody{ position, size, mass, Quat(0, 0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), initialInertiaTensor.inverse(), false });
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
