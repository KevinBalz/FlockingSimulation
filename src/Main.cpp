#include "Tako.hpp"
#include "Renderer3D.hpp"

struct Boid
{
	tako::Vector3 position;
	tako::Vector3 velocity;
};

constexpr size_t BOID_COUNT = 1000;

struct FrameData
{
	std::array<Boid, BOID_COUNT> boids;
	tako::Vector3 cameraPosition;
	tako::Quaternion cameraRotation;
};

class Game
{
public:
	void Setup(const tako::SetupData& setup)
	{
		m_renderer = new tako::Renderer3D(setup.context);
		tako::Bitmap tex(124, 124);
		tex.FillRect(0, 0, 124, 124, { 255, 255, 255, 255 });
		m_material = setup.context->CreateMaterial(&m_renderer->CreateTexture(tex));
		m_model = m_renderer->LoadModel("./Assets/boid.glb");
		for (auto& boid : prevState.boids)
		{
			boid.position = tako::Vector3(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5);
			boid.velocity = tako::Vector3(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5);
		}
	}

	void Update(tako::Input* input, float dt, FrameData* frameData)
	{
		new (frameData) FrameData();
		
		for (size_t i = 0; i < BOID_COUNT; i++)
		{
			frameData->boids[i] = SimulateBoid(prevState.boids[i], i, dt);
		}

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

		frameData->cameraPosition = prevState.cameraPosition + dt * 20 * movAxis;

		prevState = *frameData;
	}

	Boid SimulateBoid(Boid boid, size_t index, float dt)
	{
		int flockMates = 0;
		tako::Vector3 flockCenter;
		tako::Vector3 flockAvoid;
		tako::Vector3 flockSpeed;
		for (size_t i = 0; i < BOID_COUNT; i++)
		{
			if (i == index)
			{
				continue;
			}
			Boid& other = prevState.boids[i];
			auto offset = boid.position - other.position;
			auto distanceSquared = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;

			if (distanceSquared < 10 * 10)
			{
				flockMates++;
				flockCenter += other.position;
				flockSpeed += other.velocity;
				flockAvoid += offset;
			}
		}

		tako::Vector3 boundsAvoidance;
		float boundary = 20;
		float boundFactor = 1;
		auto centerOffset = tako::Vector3(0, 0, 0) - boid.position;
		auto centerDistance = centerOffset.magnitudeSquared();
		if (centerDistance > boundary * boundary)
		{
			boundsAvoidance = boundFactor / std::sqrt(centerDistance) * centerOffset;
		}
		flockCenter /= flockMates;
		flockSpeed /= flockMates;
		
		auto cohesion = 0.4f * (flockCenter - boid.position);
		auto separation = 0.1f * flockAvoid;
		auto alignment = 0.1f * flockSpeed;
		boid.velocity += dt * (cohesion + separation + alignment + boundsAvoidance);
		auto speed = boid.velocity.magnitudeSquared();
		if (speed > 5 * 5)
		{
			boid.velocity *= 5 / std::sqrt(speed);
		}
		boid.position += dt * boid.velocity;
		return boid;
	}

	void Draw(FrameData* frameData)
	{
		m_renderer->Begin();
		m_renderer->SetLightPosition({0, 0, 0});
		m_renderer->SetCameraView(tako::Matrix4::cameraViewMatrix(frameData->cameraPosition, frameData->cameraRotation));
		//auto transform = tako::Matrix4::ScaleMatrix(frameData->zoom, frameData->zoom, frameData->zoom);
		//renderer->DrawMesh(golf, texture, );

		//m_renderer->DrawModel(model, transform);

		//renderer->DrawCube(tako::Matrix4::translation(0, -1, 0), model.materials[0]);
		for (auto& boid : frameData->boids)
		{
			m_renderer->DrawModel(m_model, tako::Matrix4::translation(boid.position));
		}

		m_renderer->End();
	}
private:
	FrameData prevState;
	tako::Renderer3D* m_renderer;
	tako::Material m_material;
	tako::Model m_model;
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
