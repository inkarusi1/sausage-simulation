#include "Common/Common.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Visualization/MiniGL.h"
#include "Discregrid/geometry/TriangleMeshDistance.h"
#include "Simulation/CubicSDFCollisionDetection.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Simulation/Simulation.h"
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeManager.h"
#include "Utils/FileSystem.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"

#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

namespace
{
	struct RodPoint
	{
		Vector3r x = Vector3r::Zero();
		Vector3r oldX = Vector3r::Zero();
		Vector3r v = Vector3r::Zero();
		Real invMass = static_cast<Real>(1.0);
	};

	struct ContactAccum
	{
		Vector3r normal = Vector3r::Zero();
		Real friction = static_cast<Real>(0.0);
		unsigned int count = 0u;
	};

	DemoBase *base = nullptr;
	CubicSDFCollisionDetection *cd = nullptr;

	vector<Vector3r> slideCenter;
	vector<RodPoint> rod;
	vector<Vector3r> restLocal;
	vector<ContactAccum> contacts;

	using WallClock = std::chrono::steady_clock;
	WallClock::time_point lastWallTime = WallClock::now();
	Real simulationAccumulator = static_cast<Real>(0.0);

	const unsigned int rodPointCount = 19u;
	const unsigned int positionIterations = 10u;
	const unsigned int visualRadialSegments = 20u;
	const unsigned int slidePipeArcSegments = 28u;
	const unsigned int slidePathSamplesPerSegment = 5u;
	const unsigned int ringMajorSegments = 72u;
	const unsigned int ringTubeSegments = 18u;

	const Real pi = static_cast<Real>(3.14159265358979323846);
	const Real sausageLength = static_cast<Real>(2.75);
	const Real sausageRadius = static_cast<Real>(0.205);
	const Real sausageMassPerPoint = static_cast<Real>(0.11);
	const Real timeStepSize = static_cast<Real>(0.004);
	const Real playbackSpeed = static_cast<Real>(0.35);
	const Real collisionSkin = static_cast<Real>(0.006);
	const Real gravityY = static_cast<Real>(-9.81);
	const Real slideHorizontalShift = static_cast<Real>(-0.25);
	const Real slidePipeRadius = static_cast<Real>(0.82);
	const Real slidePipeWallThickness = static_cast<Real>(0.16);
	const Real ringMajorRadius = static_cast<Real>(0.70);
	const Real ringTubeRadius = static_cast<Real>(0.105);

	Real softness = static_cast<Real>(10.0);

	float sausageColor[4] = { 0.82f, 0.035f, 0.02f, 1.0f };
	float sausageTipColor[4] = { 0.98f, 0.08f, 0.035f, 1.0f };

	Real clampReal(const Real value, const Real minValue, const Real maxValue)
	{
		return std::max(minValue, std::min(maxValue, value));
	}

	Real softness01()
	{
		return clampReal(softness, static_cast<Real>(0.0), static_cast<Real>(100.0)) / static_cast<Real>(100.0);
	}

	Real perIterationStiffness(const Real effectiveStiffness)
	{
		const Real target = clampReal(effectiveStiffness, static_cast<Real>(0.0), static_cast<Real>(1.0));
		if (target <= static_cast<Real>(0.0))
			return static_cast<Real>(0.0);
		if (target >= static_cast<Real>(0.999))
			return static_cast<Real>(1.0);
		return static_cast<Real>(1.0) - std::pow(static_cast<Real>(1.0) - target,
			static_cast<Real>(1.0) / static_cast<Real>(positionIterations));
	}

	Real stretchStiffness()
	{
		const Real s = softness01();
		return perIterationStiffness(static_cast<Real>(0.82) + static_cast<Real>(0.18) *
			std::pow(static_cast<Real>(1.0) - s, static_cast<Real>(0.35)));
	}

	Real bendStiffness()
	{
		const Real s = softness01();
		return perIterationStiffness(static_cast<Real>(0.02) + static_cast<Real>(0.98) *
			std::pow(static_cast<Real>(1.0) - s, static_cast<Real>(2.7)));
	}

	Real shapeStiffness()
	{
		const Real s = softness01();
		return perIterationStiffness(std::pow(static_cast<Real>(1.0) - s, static_cast<Real>(4.0)));
	}

	Quaternionr rotationFromTo(const Vector3r &from, const Vector3r &to)
	{
		Vector3r f = from;
		Vector3r t = to;
		if (f.norm() < static_cast<Real>(1.0e-8) || t.norm() < static_cast<Real>(1.0e-8))
			return Quaternionr::Identity();
		f.normalize();
		t.normalize();
		Quaternionr q = Quaternionr::FromTwoVectors(f, t);
		q.normalize();
		return q;
	}

	Vector3r normalizedOr(const Vector3r &v, const Vector3r &fallback)
	{
		if (v.norm() < static_cast<Real>(1.0e-8))
			return fallback.normalized();
		return v.normalized();
	}

	Vector3r catmullRom(const Vector3r &p0, const Vector3r &p1, const Vector3r &p2, const Vector3r &p3, const Real t)
	{
		const Real t2 = t * t;
		const Real t3 = t2 * t;
		return static_cast<Real>(0.5) *
			((static_cast<Real>(2.0) * p1) +
			(-p0 + p2) * t +
			(static_cast<Real>(2.0) * p0 - static_cast<Real>(5.0) * p1 + static_cast<Real>(4.0) * p2 - p3) * t2 +
			(-p0 + static_cast<Real>(3.0) * p1 - static_cast<Real>(3.0) * p2 + p3) * t3);
	}

	void addTriangle(IndexedFaceMesh &mesh, const unsigned int a, const unsigned int b, const unsigned int c)
	{
		const unsigned int face[3] = { a, b, c };
		mesh.addFace(face);
	}

	void makeCoursePath()
	{
		slideCenter.clear();
		const Vector3r shift(slideHorizontalShift, 0.0, 0.0);
		const vector<Vector3r> controls = {
			Vector3r(-3.05, 3.35, 0.0) + shift,
			Vector3r(-2.78, 2.72, 0.0) + shift,
			Vector3r(-2.18, 2.08, 0.0) + shift,
			Vector3r(-1.18, 1.58, 0.0) + shift,
			Vector3r(0.02, 1.31, 0.0) + shift,
			Vector3r(1.12, 1.18, 0.0) + shift,
			Vector3r(1.98, 1.14, 0.0) + shift
		};

		for (unsigned int i = 0u; i + 1u < controls.size(); i++)
		{
			const Vector3r &p0 = controls[i == 0u ? i : i - 1u];
			const Vector3r &p1 = controls[i];
			const Vector3r &p2 = controls[i + 1u];
			const Vector3r &p3 = controls[std::min<unsigned int>(i + 2u, static_cast<unsigned int>(controls.size() - 1u))];
			for (unsigned int j = 0u; j < slidePathSamplesPerSegment; j++)
			{
				const Real t = static_cast<Real>(j) / static_cast<Real>(slidePathSamplesPerSegment);
				slideCenter.push_back(catmullRom(p0, p1, p2, p3, t));
			}
		}
		slideCenter.push_back(controls.back());
	}

	void loadBoxMesh(VertexData &boxVd, IndexedFaceMesh &boxMesh)
	{
		const string modelPath = base->getExePath() + "/resources/models/";
		DemoBase::loadMesh(FileSystem::normalizePath(modelPath + "cube.obj"), boxVd, boxMesh,
			Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
		boxMesh.setFlatShading(true);
	}

	unsigned int addBody(
		const VertexData &vertices,
		const IndexedFaceMesh &mesh,
		const Real density,
		const Vector3r &position,
		const Quaternionr &rotation,
		const Vector3r &scale,
		const bool dynamic,
		const Real restitution,
		const Real friction)
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
		const unsigned int index = static_cast<unsigned int>(rb.size());
		rb.push_back(new RigidBody());
		rb[index]->initBody(density, position, rotation, vertices, mesh, scale);
		if (!dynamic)
			rb[index]->setMass(static_cast<Real>(0.0));
		rb[index]->setRestitutionCoeff(restitution);
		rb[index]->setFrictionCoeff(friction);
		return index;
	}

	unsigned int addStaticVisualBody(
		const VertexData &vertices,
		const IndexedFaceMesh &mesh,
		const Vector3r &position,
		const Quaternionr &rotation,
		const Vector3r &scale,
		const Real restitution,
		const Real friction)
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
		const unsigned int index = static_cast<unsigned int>(rb.size());
		rb.push_back(new RigidBody());
		rb[index]->initBody(static_cast<Real>(1.0), position, Vector3r(1.0, 1.0, 1.0), rotation, vertices, mesh, scale);
		rb[index]->setMass(static_cast<Real>(0.0));
		rb[index]->setRestitutionCoeff(restitution);
		rb[index]->setFrictionCoeff(friction);
		return index;
	}

	void addCollisionBox(const unsigned int index, const Vector3r &scale)
	{
		const SimulationModel::RigidBodyVector &rb = Simulation::getCurrent()->getModel()->getRigidBodies();
		const vector<Vector3r> &vertices = rb[index]->getGeometry().getVertexDataLocal().getVertices();
		cd->addCollisionBox(index, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType,
			vertices.data(), static_cast<unsigned int>(vertices.size()), scale);
	}

	void buildHalfPipeMesh(VertexData &vd, IndexedFaceMesh &mesh)
	{
		const unsigned int pathCount = static_cast<unsigned int>(slideCenter.size());
		const unsigned int ringCount = slidePipeArcSegments + 1u;
		const unsigned int vertexCount = pathCount * ringCount * 2u;
		const unsigned int surfaceFaces = (pathCount - 1u) * slidePipeArcSegments * 4u;
		const unsigned int rimFaces = (pathCount - 1u) * 4u;
		const unsigned int capFaces = slidePipeArcSegments * 4u;

		vd.reserve(vertexCount);
		mesh.initMesh(vertexCount, (surfaceFaces + rimFaces + capFaces) * 2u, surfaceFaces + rimFaces + capFaces);
		mesh.setFlatShading(false);

		for (unsigned int i = 0u; i < pathCount; i++)
		{
			Vector3r tangent;
			if (i == 0u)
				tangent = slideCenter[1] - slideCenter[0];
			else if (i + 1u == pathCount)
				tangent = slideCenter[i] - slideCenter[i - 1u];
			else
				tangent = slideCenter[i + 1u] - slideCenter[i - 1u];
			tangent = normalizedOr(tangent, Vector3r(1.0, 0.0, 0.0));

			Vector3r side = Vector3r(0.0, 0.0, 1.0);
			side -= side.dot(tangent) * tangent;
			side = normalizedOr(side, Vector3r(0.0, 0.0, 1.0));
			Vector3r up = side.cross(tangent);
			up = normalizedOr(up, Vector3r(0.0, 1.0, 0.0));

			for (unsigned int j = 0u; j < ringCount; j++)
			{
				const Real theta = -static_cast<Real>(0.5) * pi + pi * static_cast<Real>(j) / static_cast<Real>(slidePipeArcSegments);
				const Vector3r radial = std::sin(theta) * side - std::cos(theta) * up;
				const Vector3r inner = slideCenter[i] + slidePipeRadius * (up + radial);
				vd.addVertex(inner);
				vd.addVertex(inner + slidePipeWallThickness * radial);
			}
		}

		const auto innerIndex = [&](const unsigned int i, const unsigned int j) { return static_cast<unsigned int>((i * ringCount + j) * 2u); };
		const auto outerIndex = [&](const unsigned int i, const unsigned int j) { return static_cast<unsigned int>((i * ringCount + j) * 2u + 1u); };

		for (unsigned int i = 0u; i + 1u < pathCount; i++)
		{
			for (unsigned int j = 0u; j < slidePipeArcSegments; j++)
			{
				const unsigned int i00 = innerIndex(i, j);
				const unsigned int i01 = innerIndex(i, j + 1u);
				const unsigned int i10 = innerIndex(i + 1u, j);
				const unsigned int i11 = innerIndex(i + 1u, j + 1u);
				addTriangle(mesh, i00, i01, i11);
				addTriangle(mesh, i00, i11, i10);

				const unsigned int o00 = outerIndex(i, j);
				const unsigned int o01 = outerIndex(i, j + 1u);
				const unsigned int o10 = outerIndex(i + 1u, j);
				const unsigned int o11 = outerIndex(i + 1u, j + 1u);
				addTriangle(mesh, o00, o10, o11);
				addTriangle(mesh, o00, o11, o01);
			}

			for (const unsigned int edge : { 0u, slidePipeArcSegments })
			{
				const unsigned int i0 = innerIndex(i, edge);
				const unsigned int i1 = innerIndex(i + 1u, edge);
				const unsigned int o0 = outerIndex(i, edge);
				const unsigned int o1 = outerIndex(i + 1u, edge);
				if (edge == 0u)
				{
					addTriangle(mesh, i0, i1, o1);
					addTriangle(mesh, i0, o1, o0);
				}
				else
				{
					addTriangle(mesh, i0, o0, o1);
					addTriangle(mesh, i0, o1, i1);
				}
			}
		}

		for (const unsigned int end : { 0u, pathCount - 1u })
		{
			for (unsigned int j = 0u; j < slidePipeArcSegments; j++)
			{
				const unsigned int i0 = innerIndex(end, j);
				const unsigned int i1 = innerIndex(end, j + 1u);
				const unsigned int o0 = outerIndex(end, j);
				const unsigned int o1 = outerIndex(end, j + 1u);
				if (end == 0u)
				{
					addTriangle(mesh, i0, o1, i1);
					addTriangle(mesh, i0, o0, o1);
				}
				else
				{
					addTriangle(mesh, i0, i1, o1);
					addTriangle(mesh, i0, o1, o0);
				}
			}
		}

		mesh.buildNeighbors();
		mesh.updateNormals(vd, 0);
		mesh.updateVertexNormals(vd);
	}

	void buildTorusMesh(VertexData &vd, IndexedFaceMesh &mesh, const Real majorRadius, const Real tubeRadius)
	{
		const unsigned int vertexCount = ringMajorSegments * ringTubeSegments;
		const unsigned int faceCount = ringMajorSegments * ringTubeSegments * 2u;

		vd.reserve(vertexCount);
		mesh.initMesh(vertexCount, faceCount * 3u, faceCount);
		mesh.setFlatShading(false);

		for (unsigned int i = 0u; i < ringMajorSegments; i++)
		{
			const Real u = static_cast<Real>(2.0) * pi * static_cast<Real>(i) / static_cast<Real>(ringMajorSegments);
			const Vector3r radial(std::cos(u), 0.0, std::sin(u));
			const Vector3r center = majorRadius * radial;
			for (unsigned int j = 0u; j < ringTubeSegments; j++)
			{
				const Real v = static_cast<Real>(2.0) * pi * static_cast<Real>(j) / static_cast<Real>(ringTubeSegments);
				const Vector3r tubeNormal = std::cos(v) * radial + std::sin(v) * Vector3r(0.0, 1.0, 0.0);
				vd.addVertex(center + tubeRadius * tubeNormal);
			}
		}

		const auto index = [&](const unsigned int i, const unsigned int j)
		{
			return (i % ringMajorSegments) * ringTubeSegments + (j % ringTubeSegments);
		};

		for (unsigned int i = 0u; i < ringMajorSegments; i++)
		{
			for (unsigned int j = 0u; j < ringTubeSegments; j++)
			{
				const unsigned int q00 = index(i, j);
				const unsigned int q10 = index(i + 1u, j);
				const unsigned int q01 = index(i, j + 1u);
				const unsigned int q11 = index(i + 1u, j + 1u);
				addTriangle(mesh, q00, q01, q11);
				addTriangle(mesh, q00, q11, q10);
			}
		}

		mesh.buildNeighbors();
		mesh.updateNormals(vd, 0);
		mesh.updateVertexNormals(vd);
	}

	CubicSDFCollisionDetection::GridPtr generateMeshSDF(
		VertexData &vd,
		IndexedFaceMesh &mesh,
		const std::array<unsigned int, 3> &resolution,
		const std::string &label)
	{
		std::vector<unsigned int> &faces = mesh.getFaces();
		const unsigned int nFaces = mesh.numFaces();

#ifdef USE_DOUBLE
		Discregrid::TriangleMesh sdfMesh(&vd.getPosition(0)[0], faces.data(), vd.size(), nFaces);
#else
		std::vector<double> doubleVertices;
		doubleVertices.resize(3u * vd.size());
		for (unsigned int i = 0u; i < vd.size(); i++)
		{
			for (unsigned int j = 0u; j < 3u; j++)
				doubleVertices[3u * i + j] = vd.getPosition(i)[j];
		}
		Discregrid::TriangleMesh sdfMesh(doubleVertices.data(), faces.data(), vd.size(), nFaces);
#endif

		Discregrid::TriangleMeshDistance meshDistance(sdfMesh);
		Eigen::AlignedBox3d domain;
		for (auto const& x : sdfMesh.vertices())
			domain.extend(x);
		domain.max() += 0.20 * Eigen::Vector3d::Ones();
		domain.min() -= 0.20 * Eigen::Vector3d::Ones();

		CubicSDFCollisionDetection::GridPtr distanceField =
			std::make_shared<CubicSDFCollisionDetection::Grid>(domain, resolution);
		auto func = Discregrid::DiscreteGrid::ContinuousFunction{};
		func = [&meshDistance](Eigen::Vector3d const& xi) { return meshDistance.signed_distance(xi).distance; };
		LOG_INFO << "Generate SDF for " << label;
		distanceField->addFunction(func, true);
		return distanceField;
	}

	void addMeshSDFCollisionObject(const unsigned int bodyIndex, CubicSDFCollisionDetection::GridPtr sdf)
	{
		const vector<Vector3r> &vertices = Simulation::getCurrent()->getModel()->getRigidBodies()[bodyIndex]->getGeometry().getVertexDataLocal().getVertices();
		cd->addCubicSDFCollisionObject(bodyIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType,
			vertices.data(), static_cast<unsigned int>(vertices.size()), sdf, Vector3r::Ones(), true, false);
	}

	void addProceduralRing(
		const Vector3r &position,
		const Quaternionr &rotation,
		const Real restitution,
		const Real friction,
		CubicSDFCollisionDetection::GridPtr ringSDF,
		const VertexData &ringVd,
		const IndexedFaceMesh &ringMesh)
	{
		const unsigned int ring = addStaticVisualBody(ringVd, ringMesh, position, rotation, Vector3r::Ones(), restitution, friction);
		addMeshSDFCollisionObject(ring, ringSDF);
	}

	void createObstacles(const VertexData &boxVd, const IndexedFaceMesh &boxMesh)
	{
		const unsigned int platform = addBody(boxVd, boxMesh, static_cast<Real>(500.0),
			Vector3r(3.0, -0.70, 0.0), Quaternionr::Identity(), Vector3r(18.0, 0.55, 6.5),
			false, static_cast<Real>(0.08), static_cast<Real>(0.90));
		addCollisionBox(platform, Vector3r(18.0, 0.55, 6.5));

		VertexData ringVd;
		IndexedFaceMesh ringMesh;
		buildTorusMesh(ringVd, ringMesh, ringMajorRadius, ringTubeRadius);
		CubicSDFCollisionDetection::GridPtr ringSDF = generateMeshSDF(ringVd, ringMesh,
			std::array<unsigned int, 3>({ 112u, 40u, 112u }), "procedural torus rings");

		addProceduralRing(Vector3r(-3.0, 4.55, 0.0), Quaternionr::Identity(),
			static_cast<Real>(0.05), static_cast<Real>(0.02), ringSDF, ringVd, ringMesh);

		const Vector3r slideExit = slideCenter.back() - slideCenter[slideCenter.size() - 2u];
		const Quaternionr ring2Rot = rotationFromTo(Vector3r(0.0, 1.0, 0.0), slideExit);
		addProceduralRing(slideCenter.back() + Vector3r(0.60, 0.42, 0.0), ring2Rot,
			static_cast<Real>(0.05), static_cast<Real>(0.02), ringSDF, ringVd, ringMesh);

		VertexData pipeVd;
		IndexedFaceMesh pipeMesh;
		buildHalfPipeMesh(pipeVd, pipeMesh);
		const unsigned int pipeBody = addStaticVisualBody(pipeVd, pipeMesh, Vector3r::Zero(), Quaternionr::Identity(), Vector3r::Ones(),
			static_cast<Real>(0.03), static_cast<Real>(0.0));
		CubicSDFCollisionDetection::GridPtr pipeSDF = generateMeshSDF(pipeVd, pipeMesh,
			std::array<unsigned int, 3>({ 128u, 64u, 64u }), "procedural half-pipe slide");
		addMeshSDFCollisionObject(pipeBody, pipeSDF);
	}

	void resetRod()
	{
		rod.clear();
		restLocal.clear();
		contacts.clear();
		rod.resize(rodPointCount);
		restLocal.resize(rodPointCount);
		contacts.resize(rodPointCount);

		const Vector3r center(-3.0, 6.25, 0.0);
		const Real restStep = sausageLength / static_cast<Real>(rodPointCount - 1u);
		const Real halfLength = static_cast<Real>(0.5) * sausageLength;

		for (unsigned int i = 0u; i < rodPointCount; i++)
		{
			const Real y = static_cast<Real>(i) * restStep - halfLength;
			restLocal[i] = Vector3r(0.0, y, 0.0);
			rod[i].x = center + restLocal[i];
			rod[i].oldX = rod[i].x;
			rod[i].v = Vector3r::Zero();
			rod[i].invMass = static_cast<Real>(1.0) / sausageMassPerPoint;
		}
	}

	void createCourseModel()
	{
		makeCoursePath();

		SimulationModel *model = Simulation::getCurrent()->getModel();
		model->cleanup();
		cd->cleanup();
		base->getSelectedParticles().clear();

		VertexData boxVd;
		IndexedFaceMesh boxMesh;
		loadBoxMesh(boxVd, boxMesh);
		createObstacles(boxVd, boxMesh);
		resetRod();

		LOG_INFO << "SausageRod softness: " << softness << "%";
		std::cout << "SausageRod softness: " << softness << "%\n";
	}

	void solveDistanceConstraint(const unsigned int a, const unsigned int b, const Real restLength, const Real stiffness)
	{
		Vector3r d = rod[b].x - rod[a].x;
		const Real len = d.norm();
		if (len < static_cast<Real>(1.0e-8))
			return;
		d /= len;
		const Real w0 = rod[a].invMass;
		const Real w1 = rod[b].invMass;
		const Real wSum = w0 + w1;
		if (wSum <= static_cast<Real>(0.0))
			return;
		const Real c = len - restLength;
		const Vector3r corr = stiffness * c / wSum * d;
		rod[a].x += w0 * corr;
		rod[b].x -= w1 * corr;
	}

	Vector3r rodCenter()
	{
		Vector3r c = Vector3r::Zero();
		for (const RodPoint &p : rod)
			c += p.x;
		return c / static_cast<Real>(rod.size());
	}

	void solveShapeMatching()
	{
		const Real stiffness = shapeStiffness();
		if (stiffness <= static_cast<Real>(0.0))
			return;

		const Vector3r c = rodCenter();
		Matrix3r a = Matrix3r::Zero();
		for (unsigned int i = 0u; i < rod.size(); i++)
			a += (rod[i].x - c) * restLocal[i].transpose();

		JacobiSVD<Matrix3r> svd(a, ComputeFullU | ComputeFullV);
		Matrix3r u = svd.matrixU();
		Matrix3r v = svd.matrixV();
		Matrix3r r = u * v.transpose();
		if (r.determinant() < static_cast<Real>(0.0))
		{
			u.col(2) *= static_cast<Real>(-1.0);
			r = u * v.transpose();
		}

		for (unsigned int i = 0u; i < rod.size(); i++)
		{
			const Vector3r goal = c + r * restLocal[i];
			rod[i].x += stiffness * (goal - rod[i].x);
		}
	}

	void solveInternalConstraints()
	{
		const Real restStep = sausageLength / static_cast<Real>(rodPointCount - 1u);
		const Real stretch = stretchStiffness();
		const Real bend = bendStiffness();

		for (unsigned int i = 0u; i + 1u < rod.size(); i++)
			solveDistanceConstraint(i, i + 1u, restStep, stretch);

		for (unsigned int span = 2u; span <= 5u; span++)
		{
			const Real spanStiffness = bend / std::sqrt(static_cast<Real>(span - 1u));
			for (unsigned int i = 0u; i + span < rod.size(); i++)
				solveDistanceConstraint(i, i + span, restStep * static_cast<Real>(span), spanStiffness);
		}

		solveShapeMatching();
	}

	void addContact(const unsigned int i, const Vector3r &normal, const Real friction)
	{
		if (i >= contacts.size())
			return;
		if (normal.squaredNorm() < static_cast<Real>(1.0e-10))
			return;
		contacts[i].normal += normal.normalized();
		contacts[i].friction = std::max(contacts[i].friction, friction);
		contacts[i].count++;
	}

	void applyCorrectionToSample(
		const unsigned int a,
		const unsigned int b,
		const Real t,
		const Vector3r &correction,
		const Vector3r &normal,
		const Real friction)
	{
		const Real wa = static_cast<Real>(1.0) - t;
		const Real wb = t;
		const Real invA = rod[a].invMass;
		const Real invB = rod[b].invMass;
		const Real denom = wa * wa * invA + wb * wb * invB;
		if (denom <= static_cast<Real>(0.0))
			return;

		rod[a].x += (wa * invA / denom) * correction;
		rod[b].x += (wb * invB / denom) * correction;
		if (wa > static_cast<Real>(0.001))
			addContact(a, normal, friction);
		if (wb > static_cast<Real>(0.001))
			addContact(b, normal, friction);
	}

	void collideSampleWithObjects(const unsigned int a, const unsigned int b, const Real t)
	{
		const Vector3r sample = (static_cast<Real>(1.0) - t) * rod[a].x + t * rod[b].x;
		SimulationModel *model = Simulation::getCurrent()->getModel();
		const SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
		const Real tolerance = sausageRadius + collisionSkin;

		for (CollisionDetection::CollisionObject *baseObject : cd->getCollisionObjects())
		{
			DistanceFieldCollisionDetection::DistanceFieldCollisionObject *co =
				dynamic_cast<DistanceFieldCollisionDetection::DistanceFieldCollisionObject *>(baseObject);
			if (co == nullptr || co->m_bodyType != CollisionDetection::CollisionObject::RigidBodyCollisionObjectType)
				continue;
			if (co->m_bodyIndex >= rb.size())
				continue;

			RigidBody *body = rb[co->m_bodyIndex];
			const Matrix3r &r = body->getTransformationR();
			const Vector3r &v1 = body->getTransformationV1();
			const Vector3r &v2 = body->getTransformationV2();
			const Vector3r local = r * (sample - body->getPosition()) + v1;

			Vector3r cpLocal;
			Vector3r nLocal;
			Real dist;
			if (co->collisionTest(local, tolerance, cpLocal, nLocal, dist))
			{
				Vector3r nWorld = r.transpose() * nLocal;
				if (nWorld.squaredNorm() < static_cast<Real>(1.0e-10))
					continue;
				nWorld.normalize();
				Vector3r correction = -dist * nWorld;
				const Real len = correction.norm();
				const Real maxCorrection = static_cast<Real>(0.12);
				if (len > maxCorrection)
					correction *= maxCorrection / len;
				applyCorrectionToSample(a, b, t, correction, nWorld, body->getFrictionCoeff());
			}
		}
	}

	void clearContacts()
	{
		for (ContactAccum &c : contacts)
		{
			c.normal.setZero();
			c.friction = static_cast<Real>(0.0);
			c.count = 0u;
		}
	}

	void solveCollisions()
	{
		for (unsigned int i = 0u; i < rod.size(); i++)
			collideSampleWithObjects(i, i, static_cast<Real>(0.0));

		const unsigned int internalSamples = 5u;
		for (unsigned int i = 0u; i + 1u < rod.size(); i++)
		{
			for (unsigned int s = 1u; s <= internalSamples; s++)
			{
				const Real t = static_cast<Real>(s) / static_cast<Real>(internalSamples + 1u);
				collideSampleWithObjects(i, i + 1u, t);
			}
		}
	}

	void applyContactVelocities()
	{
		for (unsigned int i = 0u; i < rod.size(); i++)
		{
			if (contacts[i].count == 0u)
				continue;
			Vector3r n = contacts[i].normal;
			if (n.squaredNorm() < static_cast<Real>(1.0e-10))
				continue;
			n.normalize();

			const Real vn = rod[i].v.dot(n);
			if (vn < static_cast<Real>(0.0))
				rod[i].v -= vn * n;

			const Vector3r normalV = rod[i].v.dot(n) * n;
			const Vector3r tangentV = rod[i].v - normalV;
			const Real tangentDamping = clampReal(contacts[i].friction * static_cast<Real>(0.18),
				static_cast<Real>(0.0), static_cast<Real>(0.85));
			rod[i].v = normalV + (static_cast<Real>(1.0) - tangentDamping) * tangentV;
		}
	}

	void rodSubStep()
	{
		clearContacts();
		for (RodPoint &p : rod)
		{
			p.oldX = p.x;
			p.v += timeStepSize * Vector3r(0.0, gravityY, 0.0);
			p.x += timeStepSize * p.v;
		}

		for (unsigned int iter = 0u; iter < positionIterations; iter++)
		{
			solveInternalConstraints();
			solveCollisions();
		}

		solveCollisions();

		for (RodPoint &p : rod)
			p.v = (p.x - p.oldX) / timeStepSize;
		applyContactVelocities();

		TimeManager::getCurrent()->setTime(TimeManager::getCurrent()->getTime() + timeStepSize);
	}

	void setSoftness(const Real value)
	{
		softness = clampReal(value, static_cast<Real>(0.0), static_cast<Real>(100.0));
		LOG_INFO << "SausageRod softness: " << softness << "%";
		std::cout << "SausageRod softness: " << softness << "%\n";
	}

	void changeSoftness(const Real delta)
	{
		setSoftness(softness + delta);
	}

	void reset()
	{
		Utilities::Timing::printAverageTimes();
		Utilities::Timing::reset();
		TimeManager::getCurrent()->setTime(static_cast<Real>(0.0));
		lastWallTime = WallClock::now();
		simulationAccumulator = static_cast<Real>(0.0);
		resetRod();
	}

	void timeStep()
	{
		const WallClock::time_point now = WallClock::now();
		Real wallDt = std::chrono::duration<Real>(now - lastWallTime).count();
		lastWallTime = now;

		const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
		if ((pauseAt > static_cast<Real>(0.0)) && (pauseAt < TimeManager::getCurrent()->getTime()))
			base->setValue(DemoBase::PAUSE, true);

		if (base->getValue<bool>(DemoBase::PAUSE))
		{
			simulationAccumulator = static_cast<Real>(0.0);
			return;
		}

		wallDt = clampReal(wallDt, static_cast<Real>(0.0), static_cast<Real>(0.05));
		simulationAccumulator += wallDt * playbackSpeed;

		const unsigned int maxSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
		unsigned int steps = 0u;
		while (simulationAccumulator >= timeStepSize && steps < maxSteps)
		{
			START_TIMING("SausageRodStep");
			rodSubStep();
			STOP_TIMING_AVG;
			base->step();
			simulationAccumulator -= timeStepSize;
			steps++;
		}
	}

	void buildSausageVisualMesh(
		std::vector<Vector3r> &vertices,
		std::vector<unsigned int> &faces,
		std::vector<Vector3r> &normals)
	{
		vertices.clear();
		faces.clear();
		normals.clear();
		if (rod.empty())
			return;

		vertices.reserve(rod.size() * visualRadialSegments);
		normals.reserve(rod.size() * visualRadialSegments);
		faces.reserve((rod.size() - 1u) * visualRadialSegments * 6u);

		Vector3r side = Vector3r(0.0, 0.0, 1.0);
		for (unsigned int i = 0u; i < rod.size(); i++)
		{
			Vector3r tangent;
			if (i == 0u)
				tangent = rod[1].x - rod[0].x;
			else if (i + 1u == rod.size())
				tangent = rod[i].x - rod[i - 1u].x;
			else
				tangent = rod[i + 1u].x - rod[i - 1u].x;
			tangent = normalizedOr(tangent, Vector3r(0.0, 1.0, 0.0));

			side -= side.dot(tangent) * tangent;
			if (side.norm() < static_cast<Real>(1.0e-7))
				side = std::abs(tangent.dot(Vector3r(0.0, 0.0, 1.0))) < static_cast<Real>(0.9) ?
					Vector3r(0.0, 0.0, 1.0) : Vector3r(1.0, 0.0, 0.0);
			side = normalizedOr(side, Vector3r(1.0, 0.0, 0.0));
			const Vector3r up = tangent.cross(side).normalized();

			for (unsigned int j = 0u; j < visualRadialSegments; j++)
			{
				const Real angle = static_cast<Real>(2.0) * pi * static_cast<Real>(j) / static_cast<Real>(visualRadialSegments);
				const Vector3r n = std::cos(angle) * side + std::sin(angle) * up;
				vertices.push_back(rod[i].x + sausageRadius * n);
				normals.push_back(n);
			}
		}

		const auto idx = [&](const unsigned int i, const unsigned int j)
		{
			return i * visualRadialSegments + (j % visualRadialSegments);
		};

		for (unsigned int i = 0u; i + 1u < rod.size(); i++)
		{
			for (unsigned int j = 0u; j < visualRadialSegments; j++)
			{
				const unsigned int a = idx(i, j);
				const unsigned int b = idx(i + 1u, j);
				const unsigned int c = idx(i + 1u, j + 1u);
				const unsigned int d = idx(i, j + 1u);
				faces.push_back(a);
				faces.push_back(d);
				faces.push_back(c);
				faces.push_back(a);
				faces.push_back(c);
				faces.push_back(b);
			}
		}
	}

	void drawSausage()
	{
		std::vector<Vector3r> vertices;
		std::vector<unsigned int> faces;
		std::vector<Vector3r> normals;
		buildSausageVisualMesh(vertices, faces, normals);
		if (!vertices.empty() && !faces.empty())
		{
			base->shaderBegin(sausageColor);
			MiniGL::drawMesh(vertices, faces, normals, sausageColor);
			base->shaderEnd();
		}
		if (!rod.empty())
		{
			MiniGL::drawSphere(rod.front().x, static_cast<float>(sausageRadius), sausageTipColor, 18u);
			MiniGL::drawSphere(rod.back().x, static_cast<float>(sausageRadius), sausageTipColor, 18u);
		}
	}

	void render()
	{
		base->render();
		drawSausage();
	}

	void buildModel()
	{
		TimeManager::getCurrent()->setTimeStepSize(timeStepSize);
		base->setValue(DemoBase::NUM_STEPS_PER_RENDER, 4u);
		lastWallTime = WallClock::now();
		simulationAccumulator = static_cast<Real>(0.0);
		createCourseModel();
	}
}

int main(int argc, char **argv)
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Sausage Rod Course Demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	cd = new CubicSDFCollisionDetection();
	cd->init();

	buildModel();
	base->createParameterGUI();

	MiniGL::setClientIdleFunc(timeStep);
	MiniGL::setClientSceneFunc(render);
	MiniGL::addKeyFunc('r', reset);
	MiniGL::addKeyFunc('-', []() { changeSoftness(static_cast<Real>(-10.0)); });
	MiniGL::addKeyFunc('_', []() { changeSoftness(static_cast<Real>(-10.0)); });
	MiniGL::addKeyFunc('+', []() { changeSoftness(static_cast<Real>(10.0)); });
	MiniGL::addKeyFunc('=', []() { changeSoftness(static_cast<Real>(10.0)); });
	MiniGL::addKeyFunc('0', []() { setSoftness(static_cast<Real>(0.0)); });
	MiniGL::addKeyFunc('1', []() { setSoftness(static_cast<Real>(10.0)); });
	MiniGL::addKeyFunc('5', []() { setSoftness(static_cast<Real>(50.0)); });
	MiniGL::addKeyFunc('9', []() { setSoftness(static_cast<Real>(100.0)); });
	MiniGL::setViewport(40.0f, 0.1f, 500.0f, Vector3r(-0.2, 4.1, 11.5), Vector3r(-0.3, 2.3, 0.0));

	std::cout << "SausageRodCourseDemo controls:\n"
		<< "  Space: pause/continue\n"
		<< "  r: reset current softness\n"
		<< "  +/-: softness -/+ 10%\n"
		<< "  0/1/5/9: 0%, 10%, 50%, 100% softness\n"
		<< "  New implementation: centerline rod + capsule-radius SDF collision shell.\n";

	MiniGL::mainLoop();

	base->cleanup();
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;
	delete cd;

	return 0;
}
