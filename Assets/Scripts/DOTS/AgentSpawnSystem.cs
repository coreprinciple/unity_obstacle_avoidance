using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace avoidance.dots
{
    partial struct AgentSpawnSystem : ISystem
    {
        private int _obstacleID;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<AgentSpawn>();

            _obstacleID = 0;

            foreach (var obstacle in SystemAPI.Query<RefRW<Obstacle>>())
            {
                obstacle.ValueRW.id = ++_obstacleID;
                obstacle.ValueRW.isAgent = false;
            }
        }

        private void SpawnAgent(ref SystemState state, float3 spawnPosition)
        {
            Entity spawnEntity = SystemAPI.GetSingletonEntity<AgentSpawn>();
            AgentSpawn agentSpawn = SystemAPI.GetComponent<AgentSpawn>(spawnEntity);
            NativeArray<Entity> agentEntities = state.EntityManager.Instantiate(agentSpawn.Prefab, 1, Allocator.Temp);

            for (int i = 0; i < agentEntities.Length; i++)
            {
                int id = ++_obstacleID;
                state.EntityManager.AddComponent<Agent>(agentEntities[i]);
                state.EntityManager.AddComponent<Obstacle>(agentEntities[i]);

                Agent agent = new Agent()
                {
                    id = id,
                    maxAvoidForce = 3,
                    maxSeeAhead = 1,
                    maxVelocity = 3,
                    maxSpeed = 5,
                    maxForce = 3,
                    mass = 1,
                    radius = 0.5f,
                    rotLerpSpeed = 20,
                    arrival = true,
                };
                Obstacle obstacle = new Obstacle()
                {
                    id = id,
                    position = spawnPosition,
                    isAgent = true,
                };
                SystemAPI.SetComponent(agentEntities[i], agent);
                SystemAPI.SetComponent(agentEntities[i], obstacle);

                var transform = SystemAPI.GetComponent<LocalTransform>(agentEntities[i]);
                transform.Position = spawnPosition;
                SystemAPI.SetComponent(agentEntities[i], transform);
            }
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            state.Enabled = false;

            Entity testDataEntity = state.EntityManager.CreateEntity();
            state.EntityManager.AddComponent<AgentTestDataComponent>(testDataEntity);
            SystemAPI.SetComponent(testDataEntity, new AgentTestDataComponent { targetPosition = float3.zero, isDirty = false });

            SpawnAgent(ref state, new float3(0f, 0.5f, -13f));

            for (int i = 1; i < 11; i++)
            {
                SpawnAgent(ref state, new float3(1.0f * i, 0.5f, -13f));
                SpawnAgent(ref state, new float3(-1.0f * i, 0.5f, -13f));
            }
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {

        }
    }
}

