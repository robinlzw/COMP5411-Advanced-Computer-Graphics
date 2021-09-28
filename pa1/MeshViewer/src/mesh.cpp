#include "mesh.h"
#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Sparse>
#include <queue>

typedef Eigen::Triplet<double> T;

HEdge::HEdge(bool b) {
	mBoundary = b;

	mTwin = nullptr;
	mPrev = nullptr;
	mNext = nullptr;

	mStart = nullptr;
	mFace = nullptr;

	mFlag = false;
	mValid = true;
}

HEdge* HEdge::twin() const {
	return mTwin;
}

HEdge* HEdge::setTwin(HEdge* e) {
	mTwin = e;
	return mTwin;
}

HEdge* HEdge::prev() const {
	return mPrev;
}

HEdge* HEdge::setPrev(HEdge* e) {
	mPrev = e;
	return mPrev;
}

HEdge* HEdge::next() const {
	return mNext;
}

HEdge* HEdge::setNext(HEdge* e) {
	mNext = e;
	return mNext;
}

Vertex* HEdge::start() const {
	return mStart;
}

Vertex* HEdge::setStart(Vertex* v) {
	mStart = v;
	return mStart;
}

Vertex* HEdge::end() const {
	return mNext->start();
}

Face* HEdge::leftFace() const {
	return mFace;
}

Face* HEdge::setFace(Face* f) {
	mFace = f;
	return mFace;
}

bool HEdge::flag() const {
	return mFlag;
}

bool HEdge::setFlag(bool b) {
	mFlag = b;
	return mFlag;
}

bool HEdge::isBoundary() const {
	return mBoundary;
}

bool HEdge::isValid() const {
	return mValid;
}

bool HEdge::setValid(bool b) {
	mValid = b;
	return mValid;
}

OneRingHEdge::OneRingHEdge(const Vertex* v) {
	if (v == nullptr) {
		mStart = nullptr;
		mNext = nullptr;
	} else {
		mStart = v->halfEdge();
		mNext = v->halfEdge();
	}
}

HEdge* OneRingHEdge::nextHEdge() {
	HEdge* ret = mNext;
	if (mNext != nullptr && mNext->prev()->twin() != mStart) {
		mNext = mNext->prev()->twin();
	} else {
		mNext = nullptr;
	}
	return ret;
}

OneRingVertex::OneRingVertex(const Vertex* v): ring(v) {
}

Vertex* OneRingVertex::nextVertex() {
	HEdge* he = ring.nextHEdge();
	return he != nullptr ? he->end() : nullptr;
}

Vertex::Vertex() : mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f::Zero();
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(const Eigen::Vector3f& v): mPosition(v), mHEdge(nullptr), mFlag(0) {
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(float x, float y, float z): mHEdge(nullptr), mFlag(0) {
	mPosition = Eigen::Vector3f(x, y, z);
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}


const Eigen::Vector3f& Vertex::position() const {
	return mPosition;
}

const Eigen::Vector3f& Vertex::setPosition(const Eigen::Vector3f& p) {
	mPosition = p;
	return mPosition;
}

const Eigen::Vector3f& Vertex::normal() const {
	return mNormal;
}

const Eigen::Vector3f& Vertex::setNormal(const Eigen::Vector3f& n) {
	mNormal = n;
	return mNormal;
}

const Eigen::Vector3f& Vertex::color() const {
	return mColor;
}

const Eigen::Vector3f& Vertex::setColor(const Eigen::Vector3f& c) {
	mColor = c;
	return mColor;
}

HEdge* Vertex::halfEdge() const {
	return mHEdge;
}

HEdge* Vertex::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

int Vertex::index() const {
	return mIndex;
}

int Vertex::setIndex(int i) {
	mIndex = i;
	return mIndex;
}

int Vertex::flag() const {
	return mFlag;
}

int Vertex::setFlag(int f) {
	mFlag = f;
	return mFlag;
}

bool Vertex::isValid() const {
	return mValid;
}

bool Vertex::setValid(bool b) {
	mValid = b;
	return mValid;
}
 
bool Vertex::isBoundary() const {
	OneRingHEdge ring(this);
	HEdge* curr = nullptr;
	while (curr = ring.nextHEdge()) {
		if (curr->isBoundary()) {
			return true;
		}
	}
	return false;
}

int Vertex::valence() const {
	int count = 0;
	OneRingVertex ring(this);
	Vertex* curr = nullptr;
	while (curr = ring.nextVertex()) {
		++count;
	}
	return count;
}

Face::Face() : mHEdge(nullptr), mValid(true) {
}

HEdge* Face::halfEdge() const {
	return mHEdge;
}

HEdge* Face::setHalfEdge(HEdge* he) {
	mHEdge = he;
	return mHEdge;
}

bool Face::isBoundary() const {
	HEdge* curr = mHEdge;
	do {
		if (curr->twin()->isBoundary()) {
			return true;
		}
		curr = curr->next();
	} while (curr != mHEdge);
	return false;
}

bool Face::isValid() const {
	return mValid;
}

bool Face::setValid(bool b) {
	mValid = b;
	return mValid;
}

Mesh::Mesh() {
	mVertexPosFlag = true;
	mVertexNormalFlag = true;
	mVertexColorFlag = true;
}

Mesh::~Mesh() {
	clear();
}

const std::vector< HEdge* >& Mesh::edges() const {
	return mHEdgeList;
}

const std::vector< HEdge* >& Mesh::boundaryEdges() const {
	return mBHEdgeList;
}

const std::vector< Vertex* >& Mesh::vertices() const {
	return mVertexList;
}

const std::vector< Face* >& Mesh::faces() const {
	return mFaceList;
}


bool Mesh::isVertexPosDirty() const {
	return mVertexPosFlag;
}

void Mesh::setVertexPosDirty(bool b) {
	mVertexPosFlag = b;
}

bool Mesh::isVertexNormalDirty() const {
	return mVertexNormalFlag;
}

void Mesh::setVertexNormalDirty(bool b) {
	mVertexNormalFlag = b;
}

bool Mesh::isVertexColorDirty() const {
	return mVertexColorFlag;
}

void Mesh::setVertexColorDirty(bool b) {
	mVertexColorFlag = b;
}

bool Mesh::loadMeshFile(const std::string filename) {
	// Use libigl to parse the mesh file
	bool iglFlag = igl::read_triangle_mesh(filename, mVertexMat, mFaceMat);
	if (iglFlag) {
		clear();

		// Construct the half-edge data structure.
		int numVertices = mVertexMat.rows();
		int numFaces = mFaceMat.rows();

		// Fill in the vertex list
		for (int vidx = 0; vidx < numVertices; ++vidx) {
			mVertexList.push_back(new Vertex(mVertexMat(vidx, 0),
			                                 mVertexMat(vidx, 1),
			                                 mVertexMat(vidx, 2)));
		}
		// Fill in the face list
		for (int fidx = 0; fidx < numFaces; ++fidx) {
			addFace(mFaceMat(fidx, 0), mFaceMat(fidx, 1), mFaceMat(fidx, 2));
		}

		std::vector< HEdge* > hedgeList;
		for (int i = 0; i < mBHEdgeList.size(); ++i) {
			if (mBHEdgeList[i]->start()) {
				hedgeList.push_back(mBHEdgeList[i]);
			}
			// TODO
		}
		mBHEdgeList = hedgeList;

		for (int i = 0; i < mVertexList.size(); ++i) {
			mVertexList[i]->adjHEdges.clear();
			mVertexList[i]->setIndex(i);
			mVertexList[i]->setFlag(0);
		}
	} else {
		std::cout << __FUNCTION__ << ": mesh file loading failed!\n";
	}
	return iglFlag;
}

static void _setPrevNext(HEdge* e1, HEdge* e2) {
	e1->setNext(e2);
	e2->setPrev(e1);
}

static void _setTwin(HEdge* e1, HEdge* e2) {
	e1->setTwin(e2);
	e2->setTwin(e1);
}

static void _setFace(Face* f, HEdge* e) {
	f->setHalfEdge(e);
	e->setFace(f);
}

void Mesh::addFace(int v1, int v2, int v3) {
	Face* face = new Face();

	HEdge* hedge[3];
	HEdge* bhedge[3]; // Boundary half-edges
	Vertex* vert[3];

	for (int i = 0; i < 3; ++i) {
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = mVertexList[v1];
	vert[1] = mVertexList[v2];
	vert[2] = mVertexList[v3];

	// Connect prev-next pointers
	for (int i = 0; i < 3; ++i) {
		_setPrevNext(hedge[i], hedge[(i + 1) % 3]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 3]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[2]);
	_setTwin(hedge[2], bhedge[1]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[2]);
	for (int i = 0; i < 3; ++i) {
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 3; ++i) {
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 3; ++i) {
		Vertex* start = bhedge[i]->start();
		Vertex* end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j) {
			HEdge* curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start) {
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr); // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 3; ++i) {
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

Eigen::Vector3f Mesh::initBboxMin() const {
	return (mVertexMat.colwise().minCoeff()).transpose();
}

Eigen::Vector3f Mesh::initBboxMax() const {
	return (mVertexMat.colwise().maxCoeff()).transpose();
}

void Mesh::groupingVertexFlags() {
	// Init to 255
	for (Vertex* vert : mVertexList) {
		if (vert->flag() != 0) {
			vert->setFlag(255);
		}
	}
	// Group handles
	int id = 0;
	std::vector< Vertex* > tmpList;
	for (Vertex* vert : mVertexList) {
		if (vert->flag() == 255) {
			++id;
			vert->setFlag(id);

			// Do search
			tmpList.push_back(vert);
			while (!tmpList.empty()) {
				Vertex* v = tmpList.back();
				tmpList.pop_back();

				OneRingVertex orv = OneRingVertex(v);
				while (Vertex* v2 = orv.nextVertex()) {
					if (v2->flag() == 255) {
						v2->setFlag(id);
						tmpList.push_back(v2);
					}
				}
			}
		}
	}
}

void Mesh::clear() {
	for (int i = 0; i < mHEdgeList.size(); ++i) {
		delete mHEdgeList[i];
	}
	for (int i = 0; i < mBHEdgeList.size(); ++i) {
		delete mBHEdgeList[i];
	}
	for (int i = 0; i < mVertexList.size(); ++i) {
		delete mVertexList[i];
	}
	for (int i = 0; i < mFaceList.size(); ++i) {
		delete mFaceList[i];
	}

	mHEdgeList.clear();
	mBHEdgeList.clear();
	mVertexList.clear();
	mFaceList.clear();
}

std::vector< int > Mesh::collectMeshStats() {
	int V = 0; // # of vertices
	int E = 0; // # of half-edges
	int F = 0; // # of faces
	int B = 0; // # of boundary loops
	int C = 0; // # of connected components
	int G = 0; // # of genus

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	V = mVertexList.size();
	E = mHEdgeList.size() + mBHEdgeList.size();
	F = mFaceList.size();
	// Euler formula: V - E + F = 2 (1-G)
	// boundary loops = 0
	// connected components = 1
	B = countBoundaryLoops();
	C = countConnectedComponents();
	G = (E/2 - V - F)/2 + 1;
	
	/*
	/* Collect mesh information as listed above.
	/**********************************************/

	/*====== Programming Assignment 0 ======*/

	std::vector<int>stats;
	stats.push_back(V);
	stats.push_back(E);
	stats.push_back(F);
	stats.push_back(B);
	stats.push_back(C);
	stats.push_back(G);
	return stats;
}

int Mesh::countBoundaryLoops() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/**********************************************/
	std::vector<HEdge *> remain(mBHEdgeList);
	bool isTracing = false;
	if (remain.size() == 0)
	{
		return 0;
	}

	HEdge *curr_e = remain.back();
	while (!remain.empty())
	{
		if (isTracing)
		{
			curr_e = curr_e->next();
			std::vector<HEdge *>::iterator it;
			it = std::find(remain.begin(), remain.end(), curr_e);
			if (it == remain.end())
			{
				count++;
				isTracing = false;
			}
			else
			{
				remain.erase(it);
			}
		}
		else
		{
			curr_e = remain.back();
			remain.pop_back();
			isTracing = true;
		}
	}
	/*====== Programming Assignment 0 ======*/

	return count;
}

int Mesh::countConnectedComponents() {
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/

	int V = mVertexList.size();
	std::vector<bool> visited;
	visited.reserve(V);
	for (int i=0; i<V; i++){
		visited[i] = false;
	}

	// traverse all vertices in mVertexList
	std::vector<Vertex*>::iterator it = mVertexList.begin();
	for (;it!=mVertexList.end();it++)
	{
		if (visited[(*it)->index()]) continue;

		// Use OneRingVertex for BFS
		std::queue<Vertex*> bfs_q;
		Vertex *start_v = *it;
		visited[start_v->index()] = true;
		bfs_q.push(start_v);

		while (!bfs_q.empty()){ 
			Vertex* curr_v = bfs_q.front();
			bfs_q.pop();

			OneRingVertex ring(curr_v);

			Vertex *curr_v_nb = nullptr;
			while (curr_v_nb = ring.nextVertex())
			{
				if(!visited[curr_v_nb->index()]){
					visited[curr_v_nb->index()] = true;
					bfs_q.push(curr_v_nb);
				}
			}

		}
		
		count++;
	}

	/*
	/* Helper function for Mesh::collectMeshStats()
	/* Count the number of connected components of
	/* the mesh. (Hint: use a stack)
	/**********************************************/


	/*====== Programming Assignment 0 ======*/

	return count;
}

void Mesh::computeVertexNormals() {
	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Compute per-vertex normal using neighboring
	/* facet information. (Hint: remember using a 
	/* weighting scheme. Plus, do you notice any
	/* disadvantages of your weighting scheme?)
	/**********************************************/
	std::vector<Vertex *>::iterator it = mVertexList.begin();
	for (;it!=mVertexList.end();it++){

		// iterating all neighbors
		OneRingVertex ring(*it);
		std::vector<Vertex *> nb_list;

		int nb_size = 0;
		Vertex *curr_nb = nullptr;
		while (curr_nb = ring.nextVertex())
		{
			nb_list.push_back(curr_nb);
			nb_size++;
		}

		// find all normals, just add up all normals since it is already "weighted"
		Eigen::Vector3f weighted_n(0,0,0);
		for (int i = 0; i < nb_size; i++)
		{
			Eigen::Vector3f v1 = nb_list[(i + 1) % nb_size]->position() - (*it)->position();
			Eigen::Vector3f v2 = nb_list[(i + 2) % nb_size]->position() - (*it)->position();
			Eigen::Vector3f n = v1.cross(v2);
			weighted_n += n;
		}
		weighted_n.normalize();
		(*it)->setNormal(weighted_n);

	}

	/*====== Programming Assignment 0 ======*/
	
	// Notify mesh shaders
	setVertexNormalDirty(true);
}

void Mesh::umbrellaSmooth(bool cotangentWeights)
{
	/*====== Programming Assignment 1 ======*/
	double lambda = 0.3;
	if (cotangentWeights)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 1: Implement the cotangent weighting 
		/* scheme for explicit mesh smoothing. 
		/*
		/* Hint:
		/* It is advised to double type to store the 
		/* weights to avoid numerical issues.
		/**********************************************/
		std::vector<Vertex *>::iterator it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			OneRingVertex ring(*it);
			std::vector<Vertex *> nb_list;

			int nb_size = 0;
			Vertex *curr_nb = nullptr;
			while (curr_nb = ring.nextVertex())
			{
				nb_list.push_back(curr_nb);
				nb_size++;
			}

			double W = 0;
			Eigen::Vector3f laplace(0, 0, 0);
			for (int i = 0; i < nb_size; i++)
			{
				Eigen::Vector3f v1 = (*it)->position() - nb_list[(i + 2) % nb_size]->position();
				Eigen::Vector3f v2 = nb_list[(i + 1) % nb_size]->position() - nb_list[(i + 2) % nb_size]->position();
				double sin_term_1 = (v1.cross(v2)).norm();
				double cos_term_1 = v1.dot(v2);
				double cot1  = cos_term_1 / sin_term_1;

				Eigen::Vector3f v3 = (*it)->position() - nb_list[i % nb_size]->position();
				Eigen::Vector3f v4 = nb_list[(i + 1) % nb_size]->position() - nb_list[i % nb_size]->position();
				double sin_term_2 = (v3.cross(v4)).norm();
				double cos_term_2 = v3.dot(v4);
				double cot2 = cos_term_2 / sin_term_2;
				
				W += (cot1 + cot2)/2;
				laplace += nb_list[(i+1)%nb_size]->position() * (cot1 + cot2) / 2; 
			}

			laplace = laplace / W;
			laplace = laplace - (*it)->position();
			(*it)->setPosition((*it)->position() + lambda * laplace);
		}
	}
	else
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the uniform weighting 
		/* scheme for explicit mesh smoothing.
		/**********************************************/

		std::vector<Vertex *>::iterator it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			OneRingVertex ring(*it);
			std::vector<Vertex *> nb_list;

			int nb_size = 0;
			Vertex *curr_nb = nullptr;
			while (curr_nb = ring.nextVertex())
			{
				nb_list.push_back(curr_nb);
				nb_size++;
			}

			Eigen::Vector3f laplace(0, 0, 0);
			for (int i = 0; i < nb_size; i++)
			{
				Eigen::Vector3f v1 = nb_list[i]->position() - (*it)->position();
				laplace += v1 / nb_size;
			}
			(*it)->setPosition((*it)->position() + lambda * laplace);
		}
	}

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}

void Mesh::implicitUmbrellaSmooth(bool cotangentWeights)
{

	/*====== Programming Assignment 1 ======*/
	double lambda = 0.3;

	/* A sparse linear system Ax=b solver using the conjugate gradient method. */
	auto fnConjugateGradient = [](const Eigen::SparseMatrix<float> &A,
								  const Eigen::VectorXf &b,
								  int maxIterations,
								  float errorTolerance,
								  Eigen::VectorXf &x)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Params:
		/*  A: 
		/*  b: 
		/*  maxIterations:	Max number of iterations
		/*  errorTolerance: Error tolerance for the early stopping condition
		/*  x:				Stores the final solution, but should be initialized. 
		/**********************************************/
		/*
		/* Step 1: Implement the biconjugate gradient
		/* method.
		/* Hint: https://en.wikipedia.org/wiki/Biconjugate_gradient_method
		/**********************************************/
		Eigen::VectorXf r = b - A * x;
		Eigen::VectorXf p = r;

		for (int i = 0; i < maxIterations; i++)
		{
			double alpha = r.dot(r) / (p.dot(A * p));
			Eigen::VectorXf x_next = x + alpha * p;

			// Early quit
			if (abs(x_next.norm() - x.norm()) < errorTolerance)
			{
				break;
			}

			Eigen::VectorXf r_next = r - alpha * (A * p);

			double beta = r_next.dot(r_next) / r.dot(r);
			Eigen::VectorXf p_next = r_next + beta * p;

			// update
			r = r_next;
			p = p_next;
			x = x_next;
		}
	};

	/* IMPORTANT:
	/* Please refer to the following link about the sparse matrix construction in Eigen. */
	/* http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3 */

	if (cotangentWeights)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the cotangent weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/
		// sparse matrix: [L 0 0  b: [x
		//                 0 L 0      y
		//                 0 0 L]     z]

		int V = mVertexList.size();
		Eigen::VectorXf b(3 * V);

		std::vector<Vertex *>::iterator it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			for (int i = 0; i < 3; i++)
			{
				b[(*it)->index() + i * V] = (*it)->position()[i];
			}
		}

		// sparse matrix: 1 - lambda * L, diagonal of L is -1
		std::vector<T> tripletList;
		for (int i = 0; i < 3 * V; i++)
		{
			tripletList.push_back(T(i, i, 1 + lambda));
		}

		// compute rest value in the sparse matrix: -lambda*weights
		it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			OneRingVertex ring(*it);

			std::vector<Vertex *> nb_list;

			int nb_size = 0;
			Vertex *curr_nb = nullptr;
			while (curr_nb = ring.nextVertex())
			{
				nb_list.push_back(curr_nb);
				nb_size++;
			}

			// record weights and W
			double W = 0;
			std::vector<double> weights;
			Eigen::Vector3f laplace(0, 0, 0);
			for (int i = 0; i < nb_size; i++)
			{
				Eigen::Vector3f v1 = (*it)->position() - nb_list[(i + 2) % nb_size]->position();
				Eigen::Vector3f v2 = nb_list[(i + 1) % nb_size]->position() - nb_list[(i + 2) % nb_size]->position();
				double sin_term_1 = (v1.cross(v2)).norm();
				double cos_term_1 = v1.dot(v2);
				double cot1 = cos_term_1 / sin_term_1;

				Eigen::Vector3f v3 = (*it)->position() - nb_list[i % nb_size]->position();
				Eigen::Vector3f v4 = nb_list[(i + 1) % nb_size]->position() - nb_list[i % nb_size]->position();
				double sin_term_2 = (v3.cross(v4)).norm();
				double cos_term_2 = v3.dot(v4);
				double cot2 = cos_term_2 / sin_term_2;

				W += (cot1 + cot2) / 2;
				weights.push_back((cot1 + cot2) / 2);
			}

			// update tripletList
			for (int i = 0; i < nb_size; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					tripletList.push_back(T(j * V + (*it)->index(), j * V + nb_list[(i + 1) % nb_size]->index(), -lambda * weights[i] / W));
				}
			}
		}

		// parameters setup
		Eigen::SparseMatrix<float> A(3 * V, 3 * V);
		A.setFromTriplets(tripletList.begin(), tripletList.end());
		Eigen::VectorXf x(3 * V);
		for (int i = 0; i < 3 * V; i++)
		{
			x[i] = 0.0;
		}

		// solve x
		fnConjugateGradient(A, b, 20, 1e-10, x);

		it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			Eigen::Vector3f next_position;
			for (int j = 0; j < 3; j++)
			{
				next_position[j] = x[(*it)->index() + V * j];
			}
			(*it)->setPosition(next_position);
		}
	}
	else
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 3: Implement the uniform weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/**********************************************/

		int V = mVertexList.size();
		Eigen::VectorXf b(3 * V);

		std::vector<Vertex *>::iterator it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			for (int i = 0; i < 3; i++)
			{
				b[(*it)->index() + i * V] = (*it)->position()[i];
			}
		}

		// sparse matrix: 1 - lambda * L, diagonal of L is -1
		std::vector<T> tripletList;
		for (int i = 0; i < 3 * V; i++)
		{
			tripletList.push_back(T(i, i, 1 + lambda));
		}

		// compute rest value in the sparse matrix: -lambda*1/nb
		it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			OneRingVertex ring(*it);
			std::vector<Vertex *> nb_list;

			int nb_size = 0;
			Vertex *curr_nb = nullptr;
			while (curr_nb = ring.nextVertex())
			{
				nb_list.push_back(curr_nb);
				nb_size++;
			}

			// update tripletList
			for (int i = 0; i < nb_size; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					tripletList.push_back(T(j * V + (*it)->index(), j * V + nb_list[(i + 1) % nb_size]->index(), -lambda / nb_size));
				}
			}
		}

		// parameters setup
		Eigen::SparseMatrix<float> A(3 * V, 3 * V);
		A.setFromTriplets(tripletList.begin(), tripletList.end());
		Eigen::VectorXf x(3 * V);
		for (int i = 0; i < 3 * V; i++)
		{
			x[i] = 0.0;
		}

		// solve x
		fnConjugateGradient(A, b, 20, 1e-10, x);

		it = mVertexList.begin();
		for (; it != mVertexList.end(); it++)
		{
			Eigen::Vector3f next_position;
			for (int j = 0; j < 3; j++)
			{
				next_position[j] = x[(*it)->index() + V * j];
			}
			(*it)->setPosition(next_position);
		}
	}

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}