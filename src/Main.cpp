#include "Tako.hpp"
#ifdef TAKO_IMGUI
#include "imgui.h"
#endif
#include "Reflection.hpp"
#include <memory_resource>
#include <memory>
#include <random>
#include <span>

import Tako.Renderer3D;
import Tako.Allocators.LinearAllocator;
import Flocking.Boid;
import Flocking.Octree;
import Flocking.Rect;

//constexpr size_t BOID_COUNT = 30000;


struct FrameData
{
	//StateData state;
	tako::Vector3 cameraPosition;
	tako::Quaternion cameraRotation;
	size_t transformCount;
	tako::Matrix4 boidTransforms[];
};

struct UIData
{
	int boidCount;
	int stepCount = 5000;

	REFLECT(UIData, boidCount, stepCount)
};

template<typename Callback>
void ParallelFor(size_t iterations, size_t batchSize, Callback callback)
{
	auto jobs = iterations / batchSize;
	if (jobs % batchSize != 0)
	{
		jobs++;
	}
	for (size_t j = 0; j < jobs; j++)
	{
		tako::JobSystem::JobSystem::Schedule([=]()
		{
			auto target = std::min(iterations, batchSize * (j + 1));
			for (size_t i = batchSize * j; i < target; i++)
			{
				callback(i);
			}
		});
	}
}

template<typename J, typename C>
void ScheduleContinuation(J&& j, C&& c)
{
	tako::JobSystem::Schedule([=]()
	{
		tako::JobSystem::Schedule(std::move(j));
		tako::JobSystem::Continuation(std::move(c));
	});
}
constexpr int SPAWN_RANGE = 500;

class Game
{
public:
	Game() : m_tree(Rect({0, 0, 0}, {SPAWN_RANGE * 2, SPAWN_RANGE * 2, SPAWN_RANGE * 2}))
	{
	}
	void Setup(const tako::SetupData& setup)
	{
		m_renderer = new tako::Renderer3D(setup.context);
		m_ui = setup.ui;
		tako::Bitmap tex(124, 124);
		tex.FillRect(0, 0, 124, 124, { 255, 255, 255, 255 });
		auto texture = m_renderer->CreateTexture(tex);
		m_material = setup.context->CreateMaterial(&texture);		
		#ifdef TAKO_EMSCRIPTEN
		std::string pathPrefix = "";
		#else
		std::string pathPrefix = "./Assets";
		#endif
		m_model = m_renderer->LoadModel(pathPrefix + "/boid.glb");
		for (auto& node : m_model.nodes)
		{
			node.mat = m_material;
		}

		m_uiData.boidCount = m_targetBoidCount;
		m_uiModel = m_ui->RegisterDataBinding(m_uiData);
		m_ui->LoadFont(pathPrefix + "/Aileron-Regular.otf");
		m_ui->LoadDocument(pathPrefix + "/UI.rml");
	}

	void SpawnBoids(std::span<Boid> boids)
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> spawnDistrib(-SPAWN_RANGE/2, SPAWN_RANGE/2);
		std::uniform_real_distribution<float> velDistrib(-10, 10);
		for (auto& boid : boids)
		{
			boid.position = tako::Vector3(spawnDistrib(gen), spawnDistrib(gen), spawnDistrib(gen));
			boid.velocity = tako::Vector3(velDistrib(gen), velDistrib(gen), velDistrib(gen));
			//m_tree.Insert(&boid);
		}
	}

	void Update(tako::Input* input, float dt, FrameData* frameData, size_t frameDataSize)
	{
		//new (frameData) FrameData();
		auto boidCount = (frameDataSize - sizeof(FrameData)) / sizeof(tako::Matrix4);
		
		auto prevBoids = m_prevBoids;
		auto curBoids = m_curBoids;
		if (!prevBoids || prevBoids->size() != boidCount)
		{
			m_tree.Clear();
			auto old = prevBoids;
			m_prevBoids = prevBoids = std::make_shared<std::vector<Boid>>(boidCount);	
			if (old && false)
			{
				const auto oldSize = old->size();
				for (int i = 0; i < std::min(oldSize, boidCount); i++)
				{
					(*prevBoids)[i] = (*old)[i];
				}
				if (oldSize < boidCount)
				{
					SpawnBoids(std::span{*prevBoids}.subspan(oldSize, boidCount));
				}
			}
			else
			{
				SpawnBoids(*prevBoids);
			}
			m_curBoids = curBoids = std::make_shared<std::vector<Boid>>(boidCount);

			for (auto& boid : *prevBoids)
			{
				m_tree.Insert(&boid);
			}
		}
		
		/*
		if (input->GetKey(tako::Key::N1))
		{
			boundLimiter = 1;
		}
		if (input->GetKey(tako::Key::N2))
		{
			boundLimiter = 2;
		}
		if (input->GetKey(tako::Key::N3))
		{
			boundLimiter = 3;
		}
		if (input->GetKey(tako::Key::N4))
		{
			boundLimiter = 4;
		}
		if (input->GetKey(tako::Key::N5))
		{
			boundLimiter = 5;
		}
		if (input->GetKey(tako::Key::N6))
		{
			boundLimiter = 6;
		}
		if (input->GetKey(tako::Key::N7))
		{
			boundLimiter = 7;
		}
		if (input->GetKey(tako::Key::N8))
		{
			boundLimiter = 8;
		}
		if (input->GetKey(tako::Key::N9))
		{
			boundLimiter = 9;
		}
		if (input->GetKey(tako::Key::N0))
		{
			boundLimiter = 10;
		}
		*/
		ParallelFor(boidCount, 500, [=, this](size_t i)
		{
			(*curBoids)[i] = SimulateBoid(*prevBoids, i, dt, frameData);
		});

		auto mouseMove = input->GetMousePosition();
		auto mouseDelta = mouseMove - prevMousePos;
		prevMousePos = mouseMove;

		phi = phi + mouseDelta.x * 5;
		theta = tako::mathf::clamp(theta - mouseDelta.y * 5, -90, 90);
		auto xRotation = tako::Quaternion::AngleAxis(phi, tako::Vector3(0, 1, 0));
		auto yRotation = tako::Quaternion::AngleAxis(theta, tako::Vector3(0, 0, -1));
		frameData->cameraRotation = tako::Quaternion() * xRotation * yRotation;

		tako::Vector3 movAxis;
		if (input->GetKey(tako::Key::W) || input->GetKey(tako::Key::Gamepad_Dpad_Up))
		{
			movAxis.z += 1;
		}
		if (input->GetKey(tako::Key::S) || input->GetKey(tako::Key::Gamepad_Dpad_Down))
		{
			movAxis.z -= 1;
		}
		if (input->GetKey(tako::Key::A) || input->GetKey(tako::Key::Gamepad_Dpad_Left))
		{
			movAxis.x -= 1;
		}
		if (input->GetKey(tako::Key::D) || input->GetKey(tako::Key::Gamepad_Dpad_Right))
		{
			movAxis.x += 1;
		}

		auto leftAxis = input->GetAxis(tako::Axis::Left);
		movAxis += {leftAxis.x, 0, leftAxis.y};

		movAxis = xRotation * movAxis;

		if (input->GetKey(tako::Key::Up))
		{
			movAxis.y -= 1;
		}
		if (input->GetKey(tako::Key::Down))
		{
			movAxis.y += 1;
		}
		


		frameData->cameraPosition = cameraPosition += dt * 20 * movAxis;

		/*
		#ifdef TAKO_IMGUI
		ImGui::Begin("Boids");
		ImGui::InputScalar("Target count", ImGuiDataType_U64, &m_targetBoidCount);
		ImGui::End();
		#endif
		*/

		
		if (m_uiModel.IsVariableDirty("boidCount"))
		{
			m_targetBoidCount = m_uiData.boidCount;
		}
		if (input->GetKeyDown(tako::Key::Left) || input->GetKeyDown(tako::Key::Gamepad_L))
		{
			m_targetBoidCount -= 5000;
			
		}
		if (input->GetKeyDown(tako::Key::Right) || input->GetKeyDown(tako::Key::Gamepad_R))
		{
			m_targetBoidCount += 5000;
		}

		m_targetBoidCount = std::max<size_t>(m_targetBoidCount, m_uiData.stepCount);

		if (m_uiData.boidCount != m_targetBoidCount)
		{
			m_uiData.boidCount = m_targetBoidCount;
			m_uiModel.DirtyVariable("boidCount");
		}

		tako::JobSystem::Continuation([=, this]()
		{
			frameData->transformCount = boidCount;
			auto& boids = *curBoids;
			ParallelFor(boidCount, 1000, [=, this](size_t i)
			{
				auto rotation = tako::Matrix4::DirectionToRotation(boids[i].velocity.normalized(), { 0, 1, 0 });

				frameData->boidTransforms[i] = (rotation * tako::Quaternion::FromEuler({ -90, 0, 0 }).ToRotationMatrix()).translate(boids[i].position);
			});

			m_tree.RebalanceThreaded();
			//prevState = frameData->state;
			//std::swap(m_prevBoids, m_curBoids);
			//*m_prevBoids = *m_curBoids;
			for (int i = 0; i < boidCount; i++)
			{
				(*prevBoids)[i] = (*curBoids)[i];
			}
		});
	}

	Boid SimulateBoid(std::span<Boid> prevBoids, size_t index, float dt, FrameData* frameData)
	{
		Boid* self = &prevBoids[index];
		Boid boid = *self;
		int flockMates = 0;
		tako::Vector3 flockCenter;
		tako::Vector3 flockAvoid;
		tako::Vector3 flockSpeed;
		constexpr auto lookRadius = 3;
		m_tree.Iterate({ boid.position, {lookRadius, lookRadius, lookRadius} }, [&](Boid other)
		{
			auto offset = boid.position - other.position;
			auto distanceSquared = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;

			//if distance is 0  - it probably is it self
			if (distanceSquared > 0 && distanceSquared < lookRadius * lookRadius)
			{
				flockMates++;
				flockCenter += other.position;
				flockSpeed += other.velocity;
				flockAvoid += offset / std::sqrt(distanceSquared);
			}
		});

		tako::Vector3 boundsAvoidance(0, 0, 0);
		float boundary = SPAWN_RANGE / boundLimiter;
		float boundFactor = 1;
		auto centerOffset = tako::Vector3(0, 0, 0) - boid.position;
		auto centerDistance = centerOffset.magnitudeSquared();
		if (centerDistance > boundary * boundary)
		{
			boundsAvoidance = boundFactor * centerOffset.limitMagnitude();
		}
		tako::Vector3 cohesion;
		if (flockMates > 0)
		{
			flockCenter /= flockMates;
			flockSpeed /= flockMates;
			cohesion = 4 * (flockCenter - boid.position);
		}
		
		auto separation = 15 * flockAvoid;
		auto alignment = 4 * flockSpeed;
		boid.velocity += dt * (cohesion + separation + alignment + boundsAvoidance).limitMagnitude(5);
		auto speed = boid.velocity.magnitudeSquared();
		if (speed > 10 * 10)
		{
			boid.velocity *= 10 / std::sqrt(speed);
		}
		boid.position += dt * boid.velocity;
		return boid;
	}

	void Draw(FrameData* frameData)
	{
		m_renderer->Begin();
		m_renderer->SetLightPosition(frameData->cameraPosition * -1);
		m_renderer->SetCameraView(tako::Matrix4::cameraViewMatrix(frameData->cameraPosition, frameData->cameraRotation));

		m_renderer->DrawModelInstanced(m_model, frameData->transformCount, frameData->boidTransforms);

		m_renderer->End();
	}

	void CheckFrameDataSizeChange(size_t& frameDataSize)
	{
		frameDataSize = sizeof(FrameData) * 2 + sizeof(tako::Matrix4) * m_targetBoidCount;
	}

private:
	std::vector<Boid> m_boidVecA;
	std::vector<Boid> m_boidVecB;
	std::shared_ptr<std::vector<Boid>> m_prevBoids;
	std::shared_ptr<std::vector<Boid>> m_curBoids;
	tako::Vector3 cameraPosition = { 0, 0, 0 };
	float phi = 0;
	float theta = 0;
	tako::Quaternion cameraRotation;
	tako::Vector2 prevMousePos;
	tako::Renderer3D* m_renderer;
	tako::RmlUi* m_ui;
	UIData m_uiData;
	tako::RmlUi::ModelHandle m_uiModel;
	tako::Material m_material;
	int boundLimiter = 1;
	tako::Model m_model;
	Octree m_tree;
	size_t m_targetBoidCount = 30000;
};

void Setup(void* gameData, const tako::SetupData& setup)
{
	auto game = new (gameData) Game();
	game->Setup(setup);
}

void CheckFrameDataSizeChange(void* gameData, size_t& frameDataSize)
{
	auto game = reinterpret_cast<Game*>(gameData);
	game->CheckFrameDataSizeChange(frameDataSize);
}

void Update(const tako::GameStageData stageData, tako::Input* input, float dt)
{
	auto game = reinterpret_cast<Game*>(stageData.gameData);
	game->Update(input, dt, reinterpret_cast<FrameData*>(stageData.frameData), stageData.frameDataSize);
}

void Draw(const tako::GameStageData stageData)
{
	auto game = reinterpret_cast<Game*>(stageData.gameData);
	game->Draw(reinterpret_cast<FrameData*>(stageData.frameData));
}

void tako::InitTakoConfig(GameConfig& config)
{
	config.Setup = Setup;
	config.CheckFrameDataSizeChange = CheckFrameDataSizeChange;
	config.Update = Update;
	config.Draw = Draw;
	config.graphicsAPI = tako::GraphicsAPI::WebGPU;
	config.initAudioDelayed = true;
	config.gameDataSize = sizeof(Game);
	config.frameDataSize = sizeof(FrameData);
}
