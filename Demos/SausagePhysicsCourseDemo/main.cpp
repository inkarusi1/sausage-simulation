#include "Common/Common.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Visualization.h"
#include "Discregrid/geometry/TriangleMeshDistance.h"
#include "Simulation/Constraints.h"
#include "Simulation/CubicSDFCollisionDetection.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Simulation/Simulation.h"
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeManager.h"
#include "Simulation/TimeStepController.h"
#include "Utils/FileSystem.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"

#include <algorithm>
#include <array>
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
	struct SausageState
	{
		Vector3r position = Vector3r(-3.0, 6.25, 0.0);
		Quaternionr rotation = Quaternionr::Identity();
		Vector3r velocity = Vector3r::Zero();
		Vector3r angularVelocity = Vector3r::Zero();
	};

	DemoBase *base = nullptr;
	CubicSDFCollisionDetection *cd = nullptr;
	ShapeMatchingConstraint *sausageGlobalShapeConstraint = nullptr;

	vector<Vector3r> slideCenter;

	const unsigned int invalidIndex = std::numeric_limits<unsigned int>::max();
	unsigned int sausageTetModelIndex = invalidIndex;

	const unsigned int sausageAxisSegments = 28;
	const unsigned int sausageRadialSegments = 18;
	const Real sausageLength = static_cast<Real>(2.75);
	const Real sausageVisualRadius = static_cast<Real>(0.20);
	const Real sausageParticleMass = static_cast<Real>(0.12);
	const unsigned int positionSolverIterations = 12;
	const Real slideHorizontalShift = static_cast<Real>(-0.25);
	const Real slidePipeRadius = static_cast<Real>(0.82);
	const Real slidePipeWallThickness = static_cast<Real>(0.16);
	const unsigned int slidePipeArcSegments = 28;
	const unsigned int slidePathSamplesPerSegment = 5;
	const Real ringMajorRadius = static_cast<Real>(0.70);
	const Real ringTubeRadius = static_cast<Real>(0.105);
	const unsigned int ringMajorSegments = 72;
	const unsigned int ringTubeSegments = 18;
	Real softness = static_cast<Real>(10.0);

	float sausageColor[4] = { 0.80f, 0.04f, 0.02f, 1.0f };
	float sausageTipColor[4] = { 0.98f, 0.10f, 0.03f, 1.0f };

	Real clampReal(const Real value, const Real minValue, const Real maxValue)
	{
		return std::max(minValue, std::min(maxValue, value));
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

		for (unsigned int i = 0; i + 1 < controls.size(); i++)
		{
			const Vector3r &p0 = controls[i == 0 ? i : i - 1];
			const Vector3r &p1 = controls[i];
			const Vector3r &p2 = controls[i + 1];
			const Vector3r &p3 = controls[std::min<unsigned int>(i + 2, static_cast<unsigned int>(controls.size() - 1))];
			for (unsigned int j = 0; j < slidePathSamplesPerSegment; j++)
			{
				const Real t = static_cast<Real>(j) / static_cast<Real>(slidePathSamplesPerSegment);
				slideCenter.push_back(catmullRom(p0, p1, p2, p3, t));
			}
		}
		slideCenter.push_back(controls.back());
	}

	void loadCourseMeshes(VertexData &boxVd, IndexedFaceMesh &boxMesh)
	{
		const string modelPath = base->getExePath() + "/resources/models/";
		DemoBase::loadMesh(FileSystem::normalizePath(modelPath + "cube.obj"), boxVd, boxMesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
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
		const Real friction,
		const Vector3r &velocity = Vector3r::Zero(),
		const Vector3r &angularVelocity = Vector3r::Zero())
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
		rb[index]->setVelocity(velocity);
		rb[index]->setVelocity0(velocity);
		rb[index]->setAngularVelocity(angularVelocity);
		rb[index]->setAngularVelocity0(angularVelocity);
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

	void addTriangle(IndexedFaceMesh &mesh, const unsigned int a, const unsigned int b, const unsigned int c)
	{
		const unsigned int face[3] = { a, b, c };
		mesh.addFace(face);
	}

	void addCollisionSphere(const unsigned int index, const Real radius)
	{
		const SimulationModel::RigidBodyVector &rb = Simulation::getCurrent()->getModel()->getRigidBodies();
		const vector<Vector3r> &vertices = rb[index]->getGeometry().getVertexDataLocal().getVertices();
		cd->addCollisionSphere(index, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType,
			vertices.data(), static_cast<unsigned int>(vertices.size()), radius);
	}

	void addCollisionBox(const unsigned int index, const Vector3r &scale)
	{
		const SimulationModel::RigidBodyVector &rb = Simulation::getCurrent()->getModel()->getRigidBodies();
		const vector<Vector3r> &vertices = rb[index]->getGeometry().getVertexDataLocal().getVertices();
		cd->addCollisionBox(index, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType,
			vertices.data(), static_cast<unsigned int>(vertices.size()), scale);
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
			static_cast<Real>(1.0) / static_cast<Real>(positionSolverIterations));
	}

	Real solidStiffness()
	{
		const Real s = softness01();
		const Real target = static_cast<Real>(0.02) + static_cast<Real>(0.98) *
			(static_cast<Real>(1.0) - std::pow(s, static_cast<Real>(1.5)));
		return perIterationStiffness(target);
	}

	Real solidVolumeStiffness()
	{
		const Real s = softness01();
		const Real target = static_cast<Real>(0.03) + static_cast<Real>(0.97) *
			(static_cast<Real>(1.0) - std::pow(s, static_cast<Real>(1.3)));
		return perIterationStiffness(target);
	}

	Real globalShapeMatchingStiffness()
	{
		const Real s = softness01();
		return perIterationStiffness(std::pow(static_cast<Real>(1.0) - s, static_cast<Real>(3.0)));
	}

	Real collisionTolerance()
	{
		const Real s = softness01();
		return static_cast<Real>(0.03) + static_cast<Real>(0.025) * (static_cast<Real>(1.0) - s);
	}

	void updateSoftness()
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		model->setSolidStiffness(solidStiffness());
		model->setSolidVolumeStiffness(solidVolumeStiffness());
		if (sausageGlobalShapeConstraint)
			sausageGlobalShapeConstraint->m_stiffness = globalShapeMatchingStiffness();
		if (cd)
			cd->setTolerance(collisionTolerance());
		LOG_INFO << "Sausage softness: " << softness << "%";
		std::cout << "Sausage softness: " << softness << "%\n";
	}

	void addTetCollisionObjectWithoutGeometry(const unsigned int tetModelIndex)
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		TetModel *tm = model->getTetModels()[tetModelIndex];
		ParticleData &pd = model->getParticles();
		const unsigned int offset = tm->getIndexOffset();
		const IndexedTetMesh &mesh = tm->getParticleMesh();

		cd->addCollisionObjectWithoutGeometry(tetModelIndex,
			CollisionDetection::CollisionObject::TetModelCollisionObjectType,
			&pd.getPosition(offset), mesh.numVertices(), true);

		CollisionDetection::CollisionObject *co = cd->getCollisionObjects().back();
		static_cast<DistanceFieldCollisionDetection::DistanceFieldCollisionObject*>(co)->initTetBVH(
			&pd.getPosition(offset), mesh.numVertices(), mesh.getTets().data(), mesh.numTets(), cd->getTolerance());
	}

	SausageState defaultSausageState()
	{
		SausageState state;
		state.position = Vector3r(-3.0, 6.25, 0.0);
		state.rotation = Quaternionr::Identity();
		return state;
	}

	SausageState captureSausageState()
	{
		SausageState state = defaultSausageState();
		SimulationModel *model = Simulation::getCurrent()->getModel();
		if (sausageTetModelIndex != invalidIndex && sausageTetModelIndex < model->getTetModels().size())
		{
			TetModel *tm = model->getTetModels()[sausageTetModelIndex];
			const ParticleData &pd = model->getParticles();
			const unsigned int offset = tm->getIndexOffset();
			const unsigned int nVert = tm->getParticleMesh().numVertices();
			Vector3r center = Vector3r::Zero();
			Vector3r velocity = Vector3r::Zero();
			for (unsigned int i = 0; i < nVert; i++)
			{
				center += pd.getPosition(offset + i);
				velocity += pd.getVelocity(offset + i);
			}
			center /= static_cast<Real>(nVert);
			velocity /= static_cast<Real>(nVert);

			const unsigned int sliceVertexCount = sausageRadialSegments + 1;
			Vector3r axis = pd.getPosition(offset + sausageAxisSegments * sliceVertexCount) - pd.getPosition(offset);
			if (axis.norm() < static_cast<Real>(1.0e-5))
				axis = Vector3r(0.0, 1.0, 0.0);
			state.position = center;
			state.rotation = rotationFromTo(Vector3r(0.0, 1.0, 0.0), axis);
			state.velocity = velocity;
		}
		return state;
	}

	void buildHalfPipeMesh(VertexData &vd, IndexedFaceMesh &mesh)
	{
		const Real pi = static_cast<Real>(3.14159265358979323846);
		const unsigned int pathCount = static_cast<unsigned int>(slideCenter.size());
		const unsigned int ringCount = slidePipeArcSegments + 1;
		const unsigned int vertexCount = pathCount * ringCount * 2;
		const unsigned int surfaceFaces = (pathCount - 1) * slidePipeArcSegments * 4;
		const unsigned int rimFaces = (pathCount - 1) * 4;
		const unsigned int capFaces = slidePipeArcSegments * 4;

		vd.reserve(vertexCount);
		mesh.initMesh(vertexCount, (surfaceFaces + rimFaces + capFaces) * 2, surfaceFaces + rimFaces + capFaces);
		mesh.setFlatShading(false);

		for (unsigned int i = 0; i < pathCount; i++)
		{
			Vector3r tangent;
			if (i == 0)
				tangent = slideCenter[1] - slideCenter[0];
			else if (i + 1 == pathCount)
				tangent = slideCenter[i] - slideCenter[i - 1];
			else
				tangent = slideCenter[i + 1] - slideCenter[i - 1];
			tangent = normalizedOr(tangent, Vector3r(1.0, 0.0, 0.0));

			Vector3r side = Vector3r(0.0, 0.0, 1.0);
			side -= side.dot(tangent) * tangent;
			side = normalizedOr(side, Vector3r(0.0, 0.0, 1.0));
			Vector3r up = side.cross(tangent);
			up = normalizedOr(up, Vector3r(0.0, 1.0, 0.0));

			for (unsigned int j = 0; j < ringCount; j++)
			{
				const Real theta = -static_cast<Real>(0.5) * pi + pi * static_cast<Real>(j) / static_cast<Real>(slidePipeArcSegments);
				const Vector3r radial = std::sin(theta) * side - std::cos(theta) * up;
				const Vector3r inner = slideCenter[i] + slidePipeRadius * (up + radial);
				vd.addVertex(inner);
				vd.addVertex(inner + slidePipeWallThickness * radial);
			}
		}

		const auto innerIndex = [&](const unsigned int i, const unsigned int j) { return static_cast<unsigned int>((i * ringCount + j) * 2); };
		const auto outerIndex = [&](const unsigned int i, const unsigned int j) { return static_cast<unsigned int>((i * ringCount + j) * 2 + 1); };

		for (unsigned int i = 0; i + 1 < pathCount; i++)
		{
			for (unsigned int j = 0; j < slidePipeArcSegments; j++)
			{
				const unsigned int i00 = innerIndex(i, j);
				const unsigned int i01 = innerIndex(i, j + 1);
				const unsigned int i10 = innerIndex(i + 1, j);
				const unsigned int i11 = innerIndex(i + 1, j + 1);
				addTriangle(mesh, i00, i01, i11);
				addTriangle(mesh, i00, i11, i10);

				const unsigned int o00 = outerIndex(i, j);
				const unsigned int o01 = outerIndex(i, j + 1);
				const unsigned int o10 = outerIndex(i + 1, j);
				const unsigned int o11 = outerIndex(i + 1, j + 1);
				addTriangle(mesh, o00, o10, o11);
				addTriangle(mesh, o00, o11, o01);
			}

			for (const unsigned int edge : { 0u, slidePipeArcSegments })
			{
				const unsigned int i0 = innerIndex(i, edge);
				const unsigned int i1 = innerIndex(i + 1, edge);
				const unsigned int o0 = outerIndex(i, edge);
				const unsigned int o1 = outerIndex(i + 1, edge);
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

		for (const unsigned int end : { 0u, pathCount - 1 })
		{
			for (unsigned int j = 0; j < slidePipeArcSegments; j++)
			{
				const unsigned int i0 = innerIndex(end, j);
				const unsigned int i1 = innerIndex(end, j + 1);
				const unsigned int o0 = outerIndex(end, j);
				const unsigned int o1 = outerIndex(end, j + 1);
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
		const Real pi = static_cast<Real>(3.14159265358979323846);
		const unsigned int vertexCount = ringMajorSegments * ringTubeSegments;
		const unsigned int faceCount = ringMajorSegments * ringTubeSegments * 2;

		vd.reserve(vertexCount);
		mesh.initMesh(vertexCount, faceCount * 3, faceCount);
		mesh.setFlatShading(false);

		for (unsigned int i = 0; i < ringMajorSegments; i++)
		{
			const Real u = static_cast<Real>(2.0) * pi * static_cast<Real>(i) / static_cast<Real>(ringMajorSegments);
			const Vector3r radial(std::cos(u), 0.0, std::sin(u));
			const Vector3r center = majorRadius * radial;
			for (unsigned int j = 0; j < ringTubeSegments; j++)
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

		for (unsigned int i = 0; i < ringMajorSegments; i++)
		{
			for (unsigned int j = 0; j < ringTubeSegments; j++)
			{
				const unsigned int q00 = index(i, j);
				const unsigned int q10 = index(i + 1, j);
				const unsigned int q01 = index(i, j + 1);
				const unsigned int q11 = index(i + 1, j + 1);
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
		doubleVertices.resize(3 * vd.size());
		for (unsigned int i = 0; i < vd.size(); i++)
		{
			for (unsigned int j = 0; j < 3; j++)
				doubleVertices[3 * i + j] = vd.getPosition(i)[j];
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
			std::make_shared<CubicSDFCollisionDetection::Grid>(
				domain, resolution);
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

	Real signedTetVolume(const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d)
	{
		return (b - a).cross(c - a).dot(d - a) / static_cast<Real>(6.0);
	}

	void addOrientedTet(
		std::vector<Vector3r> &points,
		std::vector<unsigned int> &tets,
		unsigned int a,
		unsigned int b,
		unsigned int c,
		unsigned int d)
	{
		if (signedTetVolume(points[a], points[b], points[c], points[d]) < static_cast<Real>(0.0))
			std::swap(b, c);
		tets.push_back(a);
		tets.push_back(b);
		tets.push_back(c);
		tets.push_back(d);
	}

	void buildSausageTetMesh(std::vector<Vector3r> &points, std::vector<unsigned int> &tets, const SausageState &state)
	{
		const Real pi = static_cast<Real>(3.14159265358979323846);
		const unsigned int sliceVertexCount = sausageRadialSegments + 1;
		const Matrix3r rotation = state.rotation.toRotationMatrix();
		const Real halfLength = static_cast<Real>(0.5) * sausageLength;

		points.clear();
		tets.clear();
		points.reserve((sausageAxisSegments + 1) * sliceVertexCount);
		tets.reserve(sausageAxisSegments * sausageRadialSegments * 3 * 4);

		for (unsigned int i = 0; i <= sausageAxisSegments; i++)
		{
			const Real alpha = static_cast<Real>(i) / static_cast<Real>(sausageAxisSegments);
			const Real y = alpha * sausageLength - halfLength;
			const Real capRoundness = static_cast<Real>(0.70) + static_cast<Real>(0.30) * std::sin(pi * alpha);
			const Real radius = sausageVisualRadius * capRoundness;
			points.push_back(state.position + rotation * Vector3r(0.0, y, 0.0));

			for (unsigned int j = 0; j < sausageRadialSegments; j++)
			{
				const Real theta = static_cast<Real>(2.0) * pi * static_cast<Real>(j) / static_cast<Real>(sausageRadialSegments);
				const Vector3r local(radius * std::cos(theta), y, radius * std::sin(theta));
				points.push_back(state.position + rotation * local);
			}
		}

		const auto centerIndex = [&](const unsigned int slice)
		{
			return slice * sliceVertexCount;
		};
		const auto ringIndex = [&](const unsigned int slice, const unsigned int segment)
		{
			return slice * sliceVertexCount + 1 + (segment % sausageRadialSegments);
		};

		for (unsigned int i = 0; i < sausageAxisSegments; i++)
		{
			for (unsigned int j = 0; j < sausageRadialSegments; j++)
			{
				const unsigned int a = centerIndex(i);
				const unsigned int b = ringIndex(i, j);
				const unsigned int c = ringIndex(i, j + 1);
				const unsigned int d = centerIndex(i + 1);
				const unsigned int e = ringIndex(i + 1, j);
				const unsigned int f = ringIndex(i + 1, j + 1);

				addOrientedTet(points, tets, a, b, c, d);
				addOrientedTet(points, tets, b, e, c, d);
				addOrientedTet(points, tets, c, e, f, d);
			}
		}
	}

	void createTetSausage(const SausageState &state)
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		std::vector<Vector3r> points;
		std::vector<unsigned int> tets;
		buildSausageTetMesh(points, tets, state);

		sausageTetModelIndex = static_cast<unsigned int>(model->getTetModels().size());
		model->addTetModel(static_cast<unsigned int>(points.size()), static_cast<unsigned int>(tets.size() / 4),
			points.data(), tets.data());

		TetModel *tm = model->getTetModels()[sausageTetModelIndex];
		ParticleData &pd = model->getParticles();
		const unsigned int offset = tm->getIndexOffset();
		const unsigned int nVert = tm->getParticleMesh().numVertices();

		tm->setInitialX(state.position);
		tm->setInitialR(state.rotation.toRotationMatrix());
		tm->setInitialScale(Vector3r::Ones());
		tm->setRestitutionCoeff(static_cast<Real>(0.04));
		tm->setFrictionCoeff(static_cast<Real>(0.03));

		for (unsigned int i = 0; i < nVert; i++)
		{
			const Vector3r relative = pd.getPosition(offset + i) - state.position;
			pd.setMass(offset + i, sausageParticleMass);
			pd.setVelocity(offset + i, state.velocity + state.angularVelocity.cross(relative));
		}

		model->addSolidConstraints(tm, 1, solidStiffness(), static_cast<Real>(0.3), solidVolumeStiffness(), false, false);

		std::vector<unsigned int> particleIndices(nVert);
		std::vector<unsigned int> numClusters(nVert, 1u);
		for (unsigned int i = 0; i < nVert; i++)
			particleIndices[i] = offset + i;
		if (model->addShapeMatchingConstraint(nVert, particleIndices.data(), numClusters.data(), globalShapeMatchingStiffness()))
			sausageGlobalShapeConstraint = static_cast<ShapeMatchingConstraint*>(model->getConstraints().back());

		tm->updateMeshNormals(pd);
		addTetCollisionObjectWithoutGeometry(sausageTetModelIndex);
		updateSoftness();
	}

	void createObstacles(const VertexData &boxVd, const IndexedFaceMesh &boxMesh)
	{
		const unsigned int platform = addBody(boxVd, boxMesh, static_cast<Real>(500.0),
			Vector3r(3.0, -0.70, 0.0), Quaternionr::Identity(), Vector3r(18.0, 0.55, 6.5),
			false, static_cast<Real>(0.15), static_cast<Real>(0.90));
		addCollisionBox(platform, Vector3r(18.0, 0.55, 6.5));

		VertexData ringVd;
		IndexedFaceMesh ringMesh;
		buildTorusMesh(ringVd, ringMesh, ringMajorRadius, ringTubeRadius);
		CubicSDFCollisionDetection::GridPtr ringSDF = generateMeshSDF(ringVd, ringMesh,
			std::array<unsigned int, 3>({ 112u, 40u, 112u }), "procedural torus rings");

		addProceduralRing(Vector3r(-3.0, 4.55, 0.0), Quaternionr::Identity(),
			static_cast<Real>(0.10), static_cast<Real>(0.02), ringSDF, ringVd, ringMesh);

		const Vector3r slideExit = slideCenter.back() - slideCenter[slideCenter.size() - 2];
		const Quaternionr ring2Rot = rotationFromTo(Vector3r(0.0, 1.0, 0.0), slideExit);
		addProceduralRing(slideCenter.back() + Vector3r(0.60, 0.28, 0.0), ring2Rot,
			static_cast<Real>(0.10), static_cast<Real>(0.02), ringSDF, ringVd, ringMesh);

		VertexData pipeVd;
		IndexedFaceMesh pipeMesh;
		buildHalfPipeMesh(pipeVd, pipeMesh);
		const unsigned int pipeBody = addStaticVisualBody(pipeVd, pipeMesh, Vector3r::Zero(), Quaternionr::Identity(), Vector3r::Ones(),
			static_cast<Real>(0.04), static_cast<Real>(0.0));
		CubicSDFCollisionDetection::GridPtr pipeSDF = generateMeshSDF(pipeVd, pipeMesh,
			std::array<unsigned int, 3>({ 128u, 64u, 64u }), "procedural half-pipe slide");
		addMeshSDFCollisionObject(pipeBody, pipeSDF);
	}

	void createCourseModel(const SausageState &state)
	{
		makeCoursePath();

		SimulationModel *model = Simulation::getCurrent()->getModel();
		model->cleanup();
		cd->cleanup();
		base->getSelectedParticles().clear();
		sausageTetModelIndex = invalidIndex;
		sausageGlobalShapeConstraint = nullptr;

		model->setContactStiffnessRigidBody(static_cast<Real>(1.0));
		model->setContactStiffnessParticleRigidBody(static_cast<Real>(100.0));
		cd->setTolerance(collisionTolerance());

		VertexData boxVd;
		IndexedFaceMesh boxMesh;
		loadCourseMeshes(boxVd, boxMesh);

		createTetSausage(state);
		createObstacles(boxVd, boxMesh);
	}

	void reset()
	{
		Utilities::Timing::printAverageTimes();
		Utilities::Timing::reset();
		TimeManager::getCurrent()->setTime(static_cast<Real>(0.0));
		createCourseModel(defaultSausageState());
	}

	void setSoftness(const Real value)
	{
		softness = clampReal(value, static_cast<Real>(0.0), static_cast<Real>(100.0));
		updateSoftness();
	}

	void changeSoftness(const Real delta)
	{
		setSoftness(softness + delta);
	}

	void timeStep()
	{
		const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
		if ((pauseAt > static_cast<Real>(0.0)) && (pauseAt < TimeManager::getCurrent()->getTime()))
			base->setValue(DemoBase::PAUSE, true);

		if (base->getValue<bool>(DemoBase::PAUSE))
			return;

		SimulationModel *model = Simulation::getCurrent()->getModel();
		const unsigned int steps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
		for (unsigned int i = 0; i < steps; i++)
		{
			START_TIMING("SausagePhysicsCourseStep");
			Simulation::getCurrent()->getTimeStep()->step(*model);
			STOP_TIMING_AVG;
			base->step();
		}

		const ParticleData &pd = model->getParticles();
		for (unsigned int i = 0; i < model->getTetModels().size(); i++)
		{
			model->getTetModels()[i]->updateMeshNormals(pd);
			model->getTetModels()[i]->updateVisMesh(pd);
		}
		for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
			model->getTriangleModels()[i]->updateMeshNormals(pd);
	}

	void drawSausageTetSurface()
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		if (sausageTetModelIndex == invalidIndex || sausageTetModelIndex >= model->getTetModels().size())
			return;

		TetModel *tm = model->getTetModels()[sausageTetModelIndex];
		base->shaderBegin(sausageColor);
		Visualization::drawMesh(model->getParticles(), tm->getSurfaceMesh(), tm->getIndexOffset(), sausageColor);
		base->shaderEnd();
	}

	void render()
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		SimulationModel::ConstraintVector &constraints = model->getConstraints();
		SimulationModel::ConstraintVector visibleConstraints;
		visibleConstraints.reserve(constraints.size());
		for (Constraint *constraint : constraints)
		{
			visibleConstraints.push_back(constraint);
		}
		constraints.swap(visibleConstraints);

		SimulationModel::TetModelVector &tetModels = model->getTetModels();
		SimulationModel::TetModelVector hiddenTetModels;
		tetModels.swap(hiddenTetModels);
		base->render();
		tetModels.swap(hiddenTetModels);

		constraints.swap(visibleConstraints);

		drawSausageTetSurface();
	}

	void buildModel()
	{
		TimeManager::getCurrent()->setTimeStepSize(static_cast<Real>(0.006));
		SimulationModel *model = Simulation::getCurrent()->getModel();
		Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd);

		TimeStepController *timeStep = static_cast<TimeStepController*>(Simulation::getCurrent()->getTimeStep());
		timeStep->setValue(TimeStepController::NUM_SUB_STEPS, 6u);
		timeStep->setValue(TimeStepController::MAX_ITERATIONS, positionSolverIterations);
		timeStep->setValue(TimeStepController::MAX_ITERATIONS_V, 10u);
		base->setValue(DemoBase::NUM_STEPS_PER_RENDER, 4u);

		createCourseModel(defaultSausageState());
	}
}

int main(int argc, char **argv)
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Sausage Physics Course Demo");

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

	std::cout << "SausagePhysicsCourseDemo controls:\n"
		<< "  Space: pause/continue\n"
		<< "  r: reset current softness\n"
		<< "  +/-: softness -/+ 10%\n"
		<< "  0/1/5/9: 0%, 10%, 50%, 100% softness\n"
		<< "  All softness values use the same native tetrahedral solid model.\n";

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
