#include "Common/Common.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Visualization/MiniGL.h"
#include "Simulation/Constraints.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Simulation/Simulation.h"
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeManager.h"
#include "Simulation/TimeStepController.h"
#include "Utils/FileSystem.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

namespace
{
	struct SpringRef
	{
		RigidBodySpring *spring;
		unsigned int span;
	};

	DemoBase *base = nullptr;
	DistanceFieldCollisionDetection *cd = nullptr;
	vector<unsigned int> sausageBodies;
	vector<SpringRef> sausageSprings;
	vector<Vector3r> slideCenter;

	const unsigned int numSausageBodies = 16;
	const Real sausageRestLength = static_cast<Real>(0.32);
	const Real sausageVisualRadius = static_cast<Real>(0.20);
	const Real sausageCollisionRadius = static_cast<Real>(0.15);
	Real softness = static_cast<Real>(10.0);

	float sausageColor[4] = { 0.82f, 0.04f, 0.02f, 1.0f };
	float sausageTipColor[4] = { 0.98f, 0.08f, 0.04f, 1.0f };
	float slideColor[4] = { 0.82f, 0.78f, 0.68f, 0.55f };
	float guideColor[4] = { 0.20f, 0.70f, 0.90f, 1.0f };

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

	void makeCoursePath()
	{
		slideCenter.clear();
		slideCenter.push_back(Vector3r(-3.05, 3.05, 0.0));
		slideCenter.push_back(Vector3r(-2.75, 2.52, 0.0));
		slideCenter.push_back(Vector3r(-2.15, 1.96, 0.0));
		slideCenter.push_back(Vector3r(-1.20, 1.46, 0.0));
		slideCenter.push_back(Vector3r(-0.10, 1.28, 0.0));
		slideCenter.push_back(Vector3r(0.95, 1.12, 0.0));
		slideCenter.push_back(Vector3r(1.70, 1.08, 0.0));
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

	void addCollisionCylinder(const unsigned int index, const Real radius, const Real height)
	{
		const SimulationModel::RigidBodyVector &rb = Simulation::getCurrent()->getModel()->getRigidBodies();
		const vector<Vector3r> &vertices = rb[index]->getGeometry().getVertexDataLocal().getVertices();
		cd->addCollisionCylinder(index, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType,
			vertices.data(), static_cast<unsigned int>(vertices.size()), Vector2r(radius, height));
	}

	void addCollisionTorus(const unsigned int index, const Real majorRadius, const Real tubeRadius)
	{
		const SimulationModel::RigidBodyVector &rb = Simulation::getCurrent()->getModel()->getRigidBodies();
		const vector<Vector3r> &vertices = rb[index]->getGeometry().getVertexDataLocal().getVertices();
		cd->addCollisionTorus(index, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType,
			vertices.data(), static_cast<unsigned int>(vertices.size()), Vector2r(majorRadius, tubeRadius));
	}

	Real springStiffness(const unsigned int span)
	{
		if (softness <= static_cast<Real>(0.0))
			return static_cast<Real>(1.0e8);

		const Real s = softness / static_cast<Real>(100.0);
		const Real hard = std::pow(static_cast<Real>(1.0) - s, static_cast<Real>(4.0));
		const Real spanScale = static_cast<Real>(1.0) / std::sqrt(static_cast<Real>(span));
		const Real baseStiffness = static_cast<Real>(40.0) + hard * static_cast<Real>(2.5e6);
		const Real longRangeBoost = span <= 2 ? static_cast<Real>(1.0) : static_cast<Real>(0.45);
		return baseStiffness * spanScale * longRangeBoost;
	}

	void updateSoftness()
	{
		for (SpringRef &ref : sausageSprings)
			ref.spring->m_stiffness = springStiffness(ref.span);

		LOG_INFO << "Softness: " << softness << "%";
		std::cout << "Softness: " << softness << "%\n";
	}

	void changeSoftness(const Real delta)
	{
		softness = clampReal(softness + delta, static_cast<Real>(0.0), static_cast<Real>(100.0));
		updateSoftness();
	}

	void setSoftnessPreset(const Real value)
	{
		softness = clampReal(value, static_cast<Real>(0.0), static_cast<Real>(100.0));
		updateSoftness();
	}

	void addSpring(const unsigned int a, const unsigned int b, const unsigned int span)
	{
		SimulationModel *model = Simulation::getCurrent()->getModel();
		const SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
		const Vector3r pa = rb[a]->getPosition();
		const Vector3r pb = rb[b]->getPosition();
		if (model->addRigidBodySpring(a, b, pa, pb, springStiffness(span)))
		{
			Constraint *constraint = model->getConstraints().back();
			sausageSprings.push_back({ static_cast<RigidBodySpring*>(constraint), span });
		}
	}

	void addSausageSprings()
	{
		sausageSprings.clear();
		for (unsigned int i = 0; i < sausageBodies.size(); i++)
		{
			for (unsigned int j = i + 1; j < sausageBodies.size(); j++)
			{
				const unsigned int span = j - i;
				if (span <= 5 || softness <= static_cast<Real>(0.0))
					addSpring(sausageBodies[i], sausageBodies[j], span);
			}
		}
	}

	void createModel()
	{
		makeCoursePath();

		SimulationModel *model = Simulation::getCurrent()->getModel();
		SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
		rb.clear();
		cd->cleanup();
		sausageBodies.clear();
		sausageSprings.clear();

		VertexData vdBox, vdSphere, vdTorus, vdCylinder;
		IndexedFaceMesh meshBox, meshSphere, meshTorus, meshCylinder;
		const string modelPath = FileSystem::normalizePath(base->getExePath() + "/resources/models/");
		DemoBase::loadMesh(modelPath + "cube.obj", vdBox, meshBox, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
		DemoBase::loadMesh(modelPath + "sphere.obj", vdSphere, meshSphere, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
		DemoBase::loadMesh(modelPath + "torus.obj", vdTorus, meshTorus, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
		DemoBase::loadMesh(modelPath + "cylinder.obj", vdCylinder, meshCylinder, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
		meshBox.setFlatShading(true);

		const unsigned int platform = addBody(vdBox, meshBox, static_cast<Real>(500.0),
			Vector3r(4.0, -0.5, 0.0), Quaternionr::Identity(), Vector3r(20.0, 1.0, 7.0),
			false, static_cast<Real>(0.2), static_cast<Real>(0.85));
		addCollisionBox(platform, Vector3r(20.0, 1.0, 7.0));

		const unsigned int ring1 = addBody(vdTorus, meshTorus, static_cast<Real>(500.0),
			Vector3r(-3.0, 4.55, 0.0), Quaternionr::Identity(), Vector3r(0.72, 0.72, 0.72),
			false, static_cast<Real>(0.15), static_cast<Real>(0.0));
		addCollisionTorus(ring1, static_cast<Real>(0.62), static_cast<Real>(0.10));

		const Quaternionr ring2Rot = rotationFromTo(Vector3r(0.0, 1.0, 0.0), Vector3r(1.0, 0.0, 0.0));
		const unsigned int ring2 = addBody(vdTorus, meshTorus, static_cast<Real>(500.0),
			Vector3r(2.15, 1.15, 0.0), ring2Rot, Vector3r(0.72, 0.72, 0.72),
			false, static_cast<Real>(0.15), static_cast<Real>(0.0));
		addCollisionTorus(ring2, static_cast<Real>(0.62), static_cast<Real>(0.10));

		for (size_t i = 1; i < slideCenter.size(); i++)
		{
			const Vector3r a = slideCenter[i - 1];
			const Vector3r b = slideCenter[i];
			const Vector3r d = b - a;
			const Real len = d.norm();
			const Vector3r mid = static_cast<Real>(0.5) * (a + b);
			const Quaternionr plankRot = rotationFromTo(Vector3r(1.0, 0.0, 0.0), d);

			const unsigned int plank = addBody(vdBox, meshBox, static_cast<Real>(500.0),
				mid + Vector3r(0.0, -0.11, 0.0), plankRot, Vector3r(static_cast<Real>(0.5) * len, static_cast<Real>(0.09), static_cast<Real>(0.62)),
				false, static_cast<Real>(0.1), static_cast<Real>(0.0));
			addCollisionBox(plank, Vector3r(static_cast<Real>(0.5) * len, static_cast<Real>(0.09), static_cast<Real>(0.62)));

			const Quaternionr railRot = rotationFromTo(Vector3r(0.0, 1.0, 0.0), d);
			for (const Real z : { static_cast<Real>(-0.67), static_cast<Real>(0.67) })
			{
				const unsigned int rail = addBody(vdCylinder, meshCylinder, static_cast<Real>(500.0),
					mid + Vector3r(0.0, 0.14, z), railRot, Vector3r(static_cast<Real>(0.10), static_cast<Real>(0.5) * len, static_cast<Real>(0.10)),
					false, static_cast<Real>(0.1), static_cast<Real>(0.0));
				addCollisionCylinder(rail, static_cast<Real>(0.10), static_cast<Real>(0.5) * len);
			}
		}

		const Vector3r head(-3.0, 6.30, 0.0);
		for (unsigned int i = 0; i < numSausageBodies; i++)
		{
			const Vector3r x = head + Vector3r(0.0, sausageRestLength * static_cast<Real>(i), 0.0);
			const unsigned int id = addBody(vdSphere, meshSphere, static_cast<Real>(120.0),
				x, Quaternionr::Identity(), sausageCollisionRadius * Vector3r::Ones(),
				true, static_cast<Real>(0.05), static_cast<Real>(0.18));
			addCollisionSphere(id, sausageCollisionRadius);
			sausageBodies.push_back(id);
		}

		addSausageSprings();
		updateSoftness();
	}

	void reset()
	{
		Utilities::Timing::printAverageTimes();
		Utilities::Timing::reset();
		Simulation::getCurrent()->reset();
		base->getSelectedParticles().clear();
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
			START_TIMING("SausageNativeStep");
			Simulation::getCurrent()->getTimeStep()->step(*model);
			STOP_TIMING_AVG;
			base->step();
		}
	}

	void drawSlideSurface()
	{
		const Real halfWidth = static_cast<Real>(0.62);
		const Real depth = static_cast<Real>(0.34);
		const unsigned int crossSegments = 8;

		for (size_t i = 1; i < slideCenter.size(); i++)
		{
			const Vector3r c0 = slideCenter[i - 1];
			const Vector3r c1 = slideCenter[i];
			for (unsigned int j = 0; j < crossSegments; j++)
			{
				const Real t0 = -static_cast<Real>(1.0) + static_cast<Real>(2.0) * static_cast<Real>(j) / static_cast<Real>(crossSegments);
				const Real t1 = -static_cast<Real>(1.0) + static_cast<Real>(2.0) * static_cast<Real>(j + 1) / static_cast<Real>(crossSegments);
				const Vector3r p00 = c0 + Vector3r(0.0, depth * t0 * t0, halfWidth * t0);
				const Vector3r p01 = c0 + Vector3r(0.0, depth * t1 * t1, halfWidth * t1);
				const Vector3r p10 = c1 + Vector3r(0.0, depth * t0 * t0, halfWidth * t0);
				const Vector3r p11 = c1 + Vector3r(0.0, depth * t1 * t1, halfWidth * t1);
				Vector3r n = (p10 - p00).cross(p01 - p00);
				if (n.norm() > static_cast<Real>(1.0e-8))
					n.normalize();
				else
					n = Vector3r(0.0, 1.0, 0.0);
				MiniGL::drawQuad(p00, p10, p11, p01, n, slideColor);
			}
		}
	}

	void render()
	{
		base->render();
		drawSlideSurface();

		const SimulationModel::RigidBodyVector &rb = Simulation::getCurrent()->getModel()->getRigidBodies();
		for (unsigned int i = 0; i + 1 < sausageBodies.size(); i++)
		{
			MiniGL::drawCylinder(rb[sausageBodies[i]]->getPosition(), rb[sausageBodies[i + 1]]->getPosition(),
				sausageColor, static_cast<float>(sausageVisualRadius), 18);
		}
		for (unsigned int i = 0; i < sausageBodies.size(); i++)
		{
			MiniGL::drawSphere(rb[sausageBodies[i]]->getPosition(), static_cast<float>(sausageVisualRadius),
				i == 0 ? sausageTipColor : sausageColor, 18);
		}

		for (size_t i = 1; i < slideCenter.size(); i++)
			MiniGL::drawCylinder(slideCenter[i - 1], slideCenter[i], guideColor, 0.01f, 6);
	}

	void buildModel()
	{
		TimeManager::getCurrent()->setTimeStepSize(static_cast<Real>(0.005));
		SimulationModel *model = Simulation::getCurrent()->getModel();
		Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd);

		TimeStepController *timeStep = static_cast<TimeStepController*>(Simulation::getCurrent()->getTimeStep());
		timeStep->setValue(TimeStepController::NUM_SUB_STEPS, 5u);
		timeStep->setValue(TimeStepController::MAX_ITERATIONS, 8u);
		timeStep->setValue(TimeStepController::MAX_ITERATIONS_V, 8u);
		createModel();
	}
}

int main(int argc, char **argv)
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Sausage Native PBD Course");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	cd = new DistanceFieldCollisionDetection();
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
	MiniGL::addKeyFunc('0', []() { setSoftnessPreset(static_cast<Real>(0.0)); });
	MiniGL::addKeyFunc('1', []() { setSoftnessPreset(static_cast<Real>(10.0)); });
	MiniGL::addKeyFunc('5', []() { setSoftnessPreset(static_cast<Real>(50.0)); });
	MiniGL::addKeyFunc('9', []() { setSoftnessPreset(static_cast<Real>(100.0)); });
	MiniGL::setViewport(40.0f, 0.1f, 500.0f, Vector3r(-0.5, 4.0, 12.0), Vector3r(-0.5, 2.4, 0.0));

	std::cout << "SausageCourseDemo native-PBD controls:\n"
		<< "  Space: pause/continue\n"
		<< "  r: reset\n"
		<< "  +/-: softness -/+ 10%\n"
		<< "  0/1/5/9: 0%, 10%, 50%, 100% softness preset\n";

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
