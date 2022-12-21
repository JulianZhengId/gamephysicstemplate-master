#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	rowNumber = 16;
	columnNumber = 16;
	depthNumber = 1;
	alpha = 0.4f;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver, Explicit_solver_3D";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "NX", TW_TYPE_INT32, &columnNumber, "min=5 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "NY", TW_TYPE_INT32, &rowNumber, "min=5 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "NZ", TW_TYPE_INT32, &depthNumber, "min=1 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alpha, "min=0.1 step=0.05 max=1.0");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver 2D!\n";
		T = new Grid(rowNumber, columnNumber);
		T = initializeGrid(T);
		break;
	case 1:
		cout << "Implicit solver 2D!\n";
		T = new Grid(rowNumber, columnNumber);
		T = initializeGrid(T);
		break;
	case 2:
		cout << "Explicit solver 3D!\n";
		T = new Grid(rowNumber, columnNumber, depthNumber);
		T = initializeGrid3D(T);

	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {//add your own parameters
	Grid* newT = new Grid(T->arr, T->r, T->c);
	for (int j = 1; j < T->r - 1; j++) {
		for (int i = 1; i < T->c - 1; i++) {
			float centralFiniteDifferenceXX = T->arr.at(j * T->c + i + 1) - 2 * T->arr.at(j * T->c + i) + T->arr.at(j * T->c + i - 1);
			float centralFiniteDifferenceYY = T->arr.at((j + 1) * T->c + i) - 2 * T->arr.at(j * T->c + i) + T->arr.at((j - 1) * T->c + i);
			newT->arr.at(j * T->c + i) = T->arr.at(j * T->c + i) + alpha * timeStep * (centralFiniteDifferenceXX + centralFiniteDifferenceYY);
		}
	}
	return newT;
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit3D(float timeStep) {
	Grid* newT = new Grid(T->arr, T->r, T->c, T->d);
	for (int k = 1; k < T->d - 1; k++) {
		for (int j = 1; j < T->r - 1; j++) {
			for (int i = 1; i < T->c - 1; i++) {
				float centralFiniteDifferenceXX = T->arr.at((k - 1) * T->r * T->c + j* T->c + i) - 2 * T->arr.at(k * T->r * T->c + j * T->c + i) + T->arr.at((k + 1) * T->r * T->c + j * T->c + i);
				float centralFiniteDifferenceYY = T->arr.at(k * T->r * T->c + (j - 1) * T->c + i) - 2 * T->arr.at(k * T->r * T->c + j * T->c + i) + T->arr.at(k * T->r * T->c + (j + 1) * T->c + i - 1);
				float centralFiniteDifferenceZZ = T->arr.at(k * T->r * T->c + j * T->c + i - 1) - 2 * T->arr.at(k * T->r * T->c + j * T->c + i) + T->arr.at(k * T->r * T->c + j * T->c + i + 1);
				newT->arr.at(k * T->r * T->c + j * T->c + i) = T->arr.at(k * T->r * T->c + j * T->c + i) + alpha * timeStep * (centralFiniteDifferenceXX + centralFiniteDifferenceYY + centralFiniteDifferenceZZ);
			}
		}
	}
	
	return newT;
}

Grid* DiffusionSimulator::initializeGrid(Grid* oldT) {
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_real_distribution<double> dist(-1.0f, 1.0f);
	for (int j = 1; j < oldT->r - 1; j++) {
		for (int i = 1; i < oldT->c - 1; i++) {
			T->arr.at(j * oldT->c + i) = dist(generator);
		}
	}
	return oldT;
}

Grid* DiffusionSimulator::initializeGrid3D(Grid* oldT) {
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_real_distribution<double> dist(-1.0f, 1.0f);
	for (int k = 1; k < oldT->d - 1; k++) {
		for (int j = 1; j < oldT->r - 1; j++) {
			for (int i = 1; i < oldT->c - 1; i++) {
				T->arr.at(k * oldT->r * oldT->c + j* oldT->c + i) = dist(generator);
			}
		}
	}
	return oldT;
}

void setupA(SparseMatrix<Real>& A, float timeStep, float alpha, int row, int column) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0

	//Identity Matrix
	for (int m = 0; m < A.index.size(); m++) {
		for (int n = 0; n < A.index.size(); n++) {
			A.index.at(m).push_back(n);
			if (m == n) {
				A.value.at(m).push_back(1.0f);
			}
			else A.value.at(m).push_back(0);
		}
	}

	float side = -1.0f * alpha; // top = bottom = left = right = -alpha / (deltaY)^2 = -alpha / (deltaX)^2
	float center = 1.0f / timeStep + 4 * alpha; // middle =  1/deltaT + 2alpha/(deltaX)^2 + 2alpha/(deltaY)^2
	for (int j = 1; j < row - 1; j++) {
		for (int i = 1; i < column - 1; i++) {
			A.set_element(j * column + i, (j - 1) * column + i, side);
			A.set_element(j * column + i, j * column + i - 1, side);
			A.set_element(j * column + i, j * column + i, center);
			A.set_element(j * column + i, j * column + i + 1, side);
			A.set_element(j * column + i, (j + 1) * column + i, side);
		}
	}
}

void setupB(std::vector<Real>& b, Grid* oldT, float timeStep) {
	float factor = 1.0f / timeStep;
	for (int i = 0; i < (oldT->r * oldT->c); i++) {
		b.at(i) = factor * oldT->arr.at(i);
	}
}

void fillT(std::vector<Real> newT, Grid* oldT) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	//for (int j = 0; j < oldT->r; j++) {
	//	for (int i = 0; i < oldT->c; i++) {
	//		//set zero to boundary
	//		if (j == 0) oldT->arr.at(j * oldT->c + i) = 0;
	//		if (i == 0) oldT->arr.at(j * oldT->c + i) = 0;
	//		if (j == oldT->r - 1) oldT->arr.at(j * oldT->c + i) = 0;
	//		if (i == oldT->c - 1) oldT->arr.at(j * oldT->c + i) = 0;

	//		//copy value
	//		oldT->arr.at(j * oldT->c + i) = newT.at(j * oldT->c + i);
	//	}
	//}

	oldT->arr = newT;
	for (int j = 0; j < oldT->r; j++) {
		//set zero to boundary
		oldT->arr.at(j * oldT->c) = 0;
		oldT->arr.at((j + 1) * oldT->c - 1) = 0;
	}

	for (int i = 0; i < oldT->c; i++) {
		//set zero to boundary
		oldT->arr.at(i) = 0;
		oldT->arr.at((oldT->r - 1) * oldT->c + i) = 0;
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters
	// solve A T = b
	// to be implemented
	//const int N = 25;//N = sizeX*sizeY*sizeZ
	const int N = T->r * T->c;
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N, N);
	std::vector<Real> *b = new std::vector<Real>(N);
	setupA(*A, timeStep, alpha, T->r, T->c);
	setupB(*b, T, timeStep);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x, T);//copy x to T
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	case 2:
		T = diffuseTemperatureExplicit3D(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	for (int m = 0; m < T->r; m++) {
		for (int n = 0; n < T->c; n++) {
			float value = T->arr.at(m * T->c + n);
			Vec3 color = Vec3(value, value, value);
			if (value <= 0) {
				color = Vec3(abs(value), 0, 0);
			}
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * color);
			DUC->drawSphere(Vec3(n * 0.05f - 0.5f, m * 0.05f - 0.5f, 0), Vec3(0.05f, 0.05f, 0.05f));
		}
	}
}

void DiffusionSimulator::drawObjects3D()
{
	//visualization
	for (int l = 1; l < T->d - 1; l++) {
		for (int m = 1; m < T->r - 1; m++) {
			for (int n = 1; n < T->c - 1; n++) {
				float value = T->arr.at(l * T->r * T->c + m * T->c + n);
				Vec3 color = Vec3(value, value, value);
				if (value <= 0) {
					color = Vec3(abs(value), 0, 0);
				}
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * color);
				DUC->drawSphere(Vec3(n * 0.05f - 0.5f, m * 0.05f - 0.5f, l * 0.05f - 0.5f), Vec3(0.05f, 0.05f, 0.05f));
			}
		}
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
		drawObjects();
		break;
	case 1:
		drawObjects();
		break;
	case 2:
		drawObjects3D();
		break;
	}
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
