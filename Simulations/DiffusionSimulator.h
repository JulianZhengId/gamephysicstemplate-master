#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(int rowNumber, int columnNumber) : r(rowNumber), c(columnNumber) {
		int dimension = rowNumber * columnNumber;
		arr = std::vector<double>(dimension, 0);
	}

	Grid(int rowNumber, int columnNumber, int depthNumber)
		: r(rowNumber), c(columnNumber), d(depthNumber) {
		int dimension = rowNumber * columnNumber * depthNumber;
		arr = std::vector<double>(dimension, 0);
	}

	Grid(std::vector<double> a, int row, int column) {
		arr = std::vector<double>(a.size(), 0);
		this->c = column;
		this->r = row;
		for (int j = 0; j < row; j++) {
			for (int i = 0; i < column; i++) {
				arr.at(j * column + i) = a.at(j * column + i);
			}
		}
	}

	Grid(std::vector<double> a, int row, int column, int depth) {
		arr = std::vector<double>(a.size(), 0);
		this->c = column;
		this->r = row;
		this->d = depth;
		for (int k = 0; k < depth; k++) {
			for (int j = 0; j < row; j++) {
				for (int i = 0; i < column; i++) {
					arr.at(k * row * column + j * column + i) = a.at(k * row * column + j * column + i);
				}
			}
		}
	}

	// Attributes
	int r;
	int c;
	int d;
	std::vector<double> arr;
};

class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);

	// Additional Functions
	Grid* initializeGrid(Grid* T);
	Grid* initializeGrid3D(Grid* T);

	void drawObjects3D();
	Grid* diffuseTemperatureExplicit3D(float timeStep);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid* T; //save results of every time step
	int columnNumber;
	int rowNumber;
	int depthNumber;
	float alpha;

	//Implicit Attributes
	//SparseMatrix<Real>* A;
	//std::vector<Real>* b
};

#endif