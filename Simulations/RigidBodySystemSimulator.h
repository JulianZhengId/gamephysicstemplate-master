#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

struct RigidBody {
public:
	Vec3 position;
	Vec3 size;
	int mass;
	Quat orientation;
	Vec3 velocity;
	Vec3 angularVelocity;
	Vec3 angularMomentum;
	Mat4 inversedInitialInertiaTensor;
	bool isFixed = false;
	std::vector<struct ForceOnBody*> forces;
};

struct ForceOnBody {
public:
	Vec3 location;
	Vec3 magnitude;
};

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	// Additional Functions
	void applyExternalForce(Vec3 force);
	void demo1Setup();
	void demo3Setup();
	void demo4Setup();
	void collisionResponse(RigidBody* A, RigidBody* B);
	Mat4 TransferMatrix(RigidBody* rb);
	void checkCollision(RigidBody* rb);
	void setIsFixed(int i, bool isFixed);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem;
	std::vector<struct RigidBody> rigidBodies;
	int bounciness = 1;
	float gravity;

	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif
