#include "Tako.hpp"
#include <memory_resource>
#include <random>

import Tako.Renderer3D;
import Tako.Allocators.LinearAllocator;
import Flocking.Boid;
import Flocking.Octree;
import Flocking.Rect;

constexpr size_t BOID_COUNT = 300000;

struct StateData
{
	std::array<Boid, BOID_COUNT> boids;
	tako::Vector3 cameraPosition = { 0, 0, 0 };
	float phi = 0;
	float theta = 0;
	tako::Quaternion cameraRotation;
};

struct FrameData
{
	StateData state;
	std::array<tako::Matrix4, BOID_COUNT> boidTransforms;
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
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> spawnDistrib(-SPAWN_RANGE/2, SPAWN_RANGE/2);
		std::uniform_real_distribution<float> velDistrib(-10, 10);
		m_renderer = new tako::Renderer3D(setup.context);
		tako::Bitmap tex(124, 124);
		tex.FillRect(0, 0, 124, 124, { 255, 255, 255, 255 });
		auto texture = m_renderer->CreateTexture(tex);
		m_material = setup.context->CreateMaterial(&texture);
		m_model = m_renderer->LoadModel("./Assets/boid.glb");
		for (auto& boid : prevState.boids)
		{
			boid.position = tako::Vector3(spawnDistrib(gen), spawnDistrib(gen), spawnDistrib(gen));
			boid.velocity = tako::Vector3(velDistrib(gen), velDistrib(gen), velDistrib(gen));
			m_tree.Insert(&boid);
		}
	}

	void Update(tako::Input* input, float dt, FrameData* frameData)
	{
		new (frameData) FrameData();
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
		ParallelFor(BOID_COUNT, 500, [=, this](size_t i)
		{
			frameData->state.boids[i] = SimulateBoid(i, dt, frameData);
		});

		auto mouseMove = input->GetMousePosition();
		auto mouseDelta = mouseMove - prevMousePos;
		prevMousePos = mouseMove;

		frameData->state.phi = prevState.phi + mouseDelta.x * 5;
		frameData->state.theta = tako::mathf::clamp(prevState.theta + mouseDelta.y * 5, -90, 90);
		auto xRotation = tako::Quaternion::AngleAxis(frameData->state.phi, tako::Vector3(0, 1, 0));
		auto yRotation = tako::Quaternion::AngleAxis(frameData->state.theta, tako::Vector3(0, 0, -1));
		frameData->state.cameraRotation = tako::Quaternion() * xRotation * yRotation;

		tako::Vector3 movAxis;
		if (input->GetKey(tako::Key::W))
		{
			movAxis.z += 1;
		}
		if (input->GetKey(tako::Key::S))
		{
			movAxis.z -= 1;
		}
		if (input->GetKey(tako::Key::A))
		{
			movAxis.x -= 1;
		}
		if (input->GetKey(tako::Key::D))
		{
			movAxis.x += 1;
		}

		movAxis = xRotation * movAxis;

		if (input->GetKey(tako::Key::Up))
		{
			movAxis.y -= 1;
		}
		if (input->GetKey(tako::Key::Down))
		{
			movAxis.y += 1;
		}


		frameData->state.cameraPosition = prevState.cameraPosition + dt * 20 * movAxis;

		tako::JobSystem::Continuation([=, this]()
		{
			ParallelFor(BOID_COUNT, 1000, [=](size_t i)
			{
				auto rotation = tako::Matrix4::DirectionToRotation(frameData->state.boids[i].velocity.normalized(), { 0, 1, 0 });

				frameData->boidTransforms[i] = (rotation * tako::Quaternion::FromEuler({ 90, 0, 0 }).ToRotationMatrix()).translate(frameData->state.boids[i].position);
			});

			m_tree.RebalanceThreaded();
			prevState = frameData->state;
		});
	}

	Boid SimulateBoid(size_t index, float dt, FrameData* frameData)
	{
		Boid* self = &prevState.boids[index];
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
		m_renderer->SetLightPosition(frameData->state.cameraPosition * -1);
		m_renderer->SetCameraView(tako::Matrix4::cameraViewMatrix(frameData->state.cameraPosition, frameData->state.cameraRotation));

		//m_renderer->DrawModelInstanced(m_model, frameData->boidTransforms.size(), frameData->boidTransforms.data());
		m_renderer->DrawCubeInstanced(m_material, frameData->boidTransforms.size(), frameData->boidTransforms.data());

		m_renderer->End();
	}
private:
	StateData prevState;
	tako::Vector2 prevMousePos;
	tako::Renderer3D* m_renderer;
	tako::Material m_material;
	int boundLimiter = 1;
	tako::Model m_model;
	Octree m_tree;
};

void Setup(void* gameData, const tako::SetupData& setup)
{
	auto game = new (gameData) Game();
	game->Setup(setup);
}

void Update(const tako::GameStageData stageData, tako::Input* input, float dt)
{
	auto game = reinterpret_cast<Game*>(stageData.gameData);
	game->Update(input, dt, reinterpret_cast<FrameData*>(stageData.frameData));
}

void Draw(const tako::GameStageData stageData)
{
	auto game = reinterpret_cast<Game*>(stageData.gameData);
	game->Draw(reinterpret_cast<FrameData*>(stageData.frameData));
}

void tako::InitTakoConfig(GameConfig& config)
{
	config.Setup = Setup;
	config.Update = Update;
	config.Draw = Draw;
	config.graphicsAPI = tako::GraphicsAPI::WebGPU;
	config.initAudioDelayed = true;
	config.gameDataSize = sizeof(Game);
	config.frameDataSize = sizeof(FrameData);
}
