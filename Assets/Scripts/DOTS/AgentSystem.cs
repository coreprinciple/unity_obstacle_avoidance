using Unity.Burst;
using Unity.Physics;
using Unity.Entities;
using Unity.Collections;

namespace avoidance.dots
{
    partial struct AgentSystem : ISystem
    {
        private NativeList<Obstacle> _obstacles;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<Agent>();
            _obstacles = new NativeList<Obstacle>(100, Allocator.Persistent);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            var testData = SystemAPI.GetSingleton<AgentTestDataComponent>();
            
            var obstacleQuery = SystemAPI.QueryBuilder().WithAll<Obstacle>().Build();
            var obstacles = obstacleQuery.ToComponentDataArray<Obstacle>(Allocator.Temp);
            float deltaTime = SystemAPI.Time.DeltaTime;

            _obstacles.Clear();
            _obstacles.AddRange(obstacles);

            new AgentJob
            {
                physicsWorld = physicsWorld,
                obstacles = _obstacles,
                targetPosition = testData.targetPosition,
                deltaTime = deltaTime,
                isDirty = testData.isDirty,
            }.ScheduleParallel();

            obstacles.Dispose();
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {
            _obstacles.Dispose();
        }

    }
}
