#include "Tako.hpp"
#include "Renderer3D.hpp"
#include <memory_resource>
#include <map>
#include "Allocators/LinearAllocator.hpp"
#include "Boid.hpp"
#include "Octree.hpp"

constexpr size_t BOID_COUNT = 3000;

struct StateData
{
	std::array<Boid, BOID_COUNT> boids;
	tako::Vector3 cameraPosition = { 0, 0, -200};
	float phi = 0;
	float theta = 0;
	tako::Quaternion cameraRotation;
};

struct FrameData
{
	StateData state;
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
constexpr int SPAWN_RANGE = 300;

class Game
{
public:
	Game() : m_tree(Rect({ 0, 0, 0 }, {SPAWN_RANGE, SPAWN_RANGE, SPAWN_RANGE }))
	{
	}
	void Setup(const tako::SetupData& setup)
	{
		m_renderer = new tako::Renderer3D(setup.context);
		tako::Bitmap tex(124, 124);
		tex.FillRect(0, 0, 124, 124, { 255, 255, 255, 255 });
		m_material = setup.context->CreateMaterial(&m_renderer->CreateTexture(tex));
		m_model = m_renderer->LoadModel("./Assets/boid.glb");
		for (auto& boid : prevState.boids)
		{
			boid.position = tako::Vector3(rand() % SPAWN_RANGE - SPAWN_RANGE / 2, rand() % SPAWN_RANGE - SPAWN_RANGE / 2, rand() % SPAWN_RANGE - SPAWN_RANGE / 2);
			boid.velocity = tako::Vector3(rand() % SPAWN_RANGE - SPAWN_RANGE / 2, rand() % SPAWN_RANGE - SPAWN_RANGE / 2, rand() % SPAWN_RANGE - SPAWN_RANGE / 2);
			m_tree.Insert(&boid);
		}
	}

	void Update(tako::Input* input, float dt, FrameData* frameData)
	{
		new (frameData) FrameData();
		tako::JobSystem::Schedule([this, frameData, dt]()
		{
			m_tree.RebalanceThreaded();
			tako::JobSystem::Continuation([=]()
			{
				//LOG("{}", m_tree.GetElementCount());
				ParallelFor(BOID_COUNT, 1, [=](size_t i)
				{
					frameData->state.boids[i] = SimulateBoid(i, dt, frameData);
				});
			});
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

		tako::JobSystem::Continuation([=]()
		{
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
		//auto range = m_cellMap.equal_range(m_hashes[index]);
		m_tree.Iterate({ boid.position, {10, 10, 10} }, [&](Boid other, Boid* ref)
		{
			if (self == ref)
			{
				return;
			}
			auto offset = boid.position - other.position;
			auto distanceSquared = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;

			if (distanceSquared < 10 * 10)
			{
				flockMates++;
				flockCenter += other.position;
				flockSpeed += other.velocity;
				if (distanceSquared < 5 * 5)
				{
					flockAvoid += offset;
				}

			}
		});

		tako::Vector3 boundsAvoidance;
		float boundary = 100;
		float boundFactor = 2;
		auto centerOffset = tako::Vector3(0, 0, 0) - boid.position;
		auto centerDistance = centerOffset.magnitudeSquared();
		if (centerDistance > boundary * boundary)
		{
			boundsAvoidance = boundFactor / std::sqrt(centerDistance) * centerOffset;
		}
		if (flockMates > 0)
		{
			flockCenter /= flockMates;
			flockSpeed /= flockMates;
		}
		
		auto cohesion = 1 * (flockCenter - boid.position);
		auto separation = 2 * flockAvoid;
		auto alignment = 1.5f * flockSpeed;
		boid.velocity += dt * (cohesion + separation + alignment + boundsAvoidance).limitMagnitude();
		auto speed = boid.velocity.magnitudeSquared();
		if (speed > 4 * 4)
		{
			boid.velocity *= 4 / std::sqrt(speed);
		}
		boid.position += dt * boid.velocity;
		return boid;
	}

	void Draw(FrameData* frameData)
	{
		m_renderer->Begin();
		m_renderer->SetLightPosition({0, 0, 0});
		m_renderer->SetCameraView(tako::Matrix4::cameraViewMatrix(frameData->state.cameraPosition, frameData->state.cameraRotation));

		for (auto& boid : frameData->state.boids)
		{
			m_renderer->DrawModel(m_model, tako::Matrix4::translation(boid.position));
		}

		m_renderer->End();
	}
private:
	StateData prevState;
	tako::Vector2 prevMousePos;
	tako::Renderer3D* m_renderer;
	tako::Material m_material;
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
	config.graphicsAPI = tako::GraphicsAPI::Vulkan;
	config.gameDataSize = sizeof(Game);
	config.frameDataSize = sizeof(FrameData);
}
