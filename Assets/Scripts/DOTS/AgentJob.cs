using Unity.Burst;
using Unity.Physics;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;

namespace avoidance.dots
{
    [BurstCompile]
    public partial struct AgentJob : IJobEntity
    {
        [ReadOnly] public PhysicsWorld physicsWorld;
        [ReadOnly] public NativeList<Obstacle> obstacles;
        [ReadOnly] public float3 targetPosition;
        [ReadOnly] public float deltaTime;
        [ReadOnly] public bool isDirty;

        private bool LineIntersectsCircle(float3 ahead, float3 ahead2, float3 point, float radius)
        {
            return math.distance(point, ahead) <= radius || math.distance(point, ahead2) <= radius;
        }

        private int GetOverlapAgent(NativeList<Obstacle> obstacles, Agent agent, float3 position)
        {
            int overlapAgent = -1;
            float distance = float.PositiveInfinity;

            for (int i = 0; i < obstacles.Length; i++)
            {
                if (!obstacles[i].isAgent || obstacles[i].id == agent.id)
                    continue;

                float d = math.distance(obstacles[i].position, position);
                if (d < agent.radius && d < distance)
                {
                    overlapAgent = obstacles[i].id;
                    distance = d;
                }
            }
            return overlapAgent;
        }

        private int FindMostThreateningObstacle(int agentID, NativeList<Obstacle> obstacles, float3 position, float3 ahead, float3 ahead2, float radius)
        {
            int mostThreatening = -1;

            for (int i = 0; i < obstacles.Length; i++)
            {
                if (agentID == obstacles[i].id)
                    continue;

                float3 obstaclePos = obstacles[i].position;
                bool collision = LineIntersectsCircle(ahead, ahead2, obstaclePos, radius);

                if (collision == false)
                    continue;

                if (mostThreatening == -1)
                    mostThreatening = i;
                else if (math.distance(position, obstaclePos) < math.distance(position, obstacles[mostThreatening].position))
                    mostThreatening = i;
            }
            return mostThreatening;
        }

        private float3 CollisionAvoidance(NativeList<Obstacle> obstacles, Agent agent, float3 position, float3 velocity)
        {
            float3 seeAhead = math.normalize(velocity) * agent.maxSeeAhead;
            float3 ahead = position + seeAhead;
            float3 ahead2 = position + seeAhead * 0.5f;

            int mostThreatening = FindMostThreateningObstacle(agent.id, obstacles, position, ahead, ahead2, agent.radius);
            float3 avoidance = float3.zero;

            if (mostThreatening > -1)
            {
                float3 obstaclePos = obstacles[mostThreatening].position;
                obstaclePos.y = position.y;

                float dist = math.distance(position, obstaclePos);
                float ratio = (AvoidanceCommon.AVOID_DIST - math.min(dist, AvoidanceCommon.AVOID_DIST)) / AvoidanceCommon.AVOID_DIST;

                avoidance.x = ahead.x - obstacles[mostThreatening].position.x;
                avoidance.z = ahead.z - obstacles[mostThreatening].position.z;
                avoidance = math.normalize(avoidance) * agent.maxAvoidForce * ratio;
            }
            return avoidance;
        }

        private float3 Seek(Agent agent, float3 position, float3 targetPos, float deltaTime)
        {
            float3 pos = position;
            float3 desiredVelocity = math.normalize(targetPos - pos) * agent.maxVelocity;
            return math.normalize(desiredVelocity - agent.velocity) * agent.maxForce;
        }

        private float3 CollisionRaycast(Agent agent, float3 position, quaternion rotation, float3 direction)
        {
            float3 deltaPos = float3.zero;
            NativeList<RaycastHit> raycastHits = new NativeList<RaycastHit>(Allocator.TempJob);

            for (int i = 0; i < AvoidanceCommon.NUM_RAYS; ++i)
            {
                raycastHits.Clear();

                float radian = ((i / ((float)AvoidanceCommon.NUM_RAYS - 1)) * AvoidanceCommon.RAY_ANGLE * 2 - AvoidanceCommon.RAY_ANGLE) * math.TORADIANS;

                quaternion rotMode = quaternion.AxisAngle(new float3(1, 0, 0), radian);
                quaternion rot = math.mul(rotation, rotMode);
                float3 rayDir = math.normalize(math.Euler(rotation) + math.Euler(rotMode));

                RaycastInput raycastInput = new RaycastInput
                {
                    Start = position,
                    End = position + rayDir * AvoidanceCommon.RAY_RANGE,
                    Filter = CollisionFilter.Default
                };

                physicsWorld.CastRay(raycastInput, ref raycastHits);

                if (raycastHits.Length > 1)
                    deltaPos -= (1.0f / AvoidanceCommon.NUM_RAYS) * agent.maxVelocity * rayDir;
                else
                    deltaPos += (1.0f / AvoidanceCommon.NUM_RAYS) * agent.maxVelocity * rayDir;
            }
            raycastHits.Dispose();
            return deltaPos;
        }

        private void UpdateMove(NativeList<Obstacle> obstacles, ref AgentAspect agentAsepct, float3 targetPosition, float deltaTime)
        {
            RefRW<Agent> agent = agentAsepct.agent;
            RefRW<Obstacle> agentObstacle = agentAsepct.obstacle;
            RefRW<LocalTransform> transform = agentAsepct.localTransform;

            float3 position = transform.ValueRO.Position;
            float3 targetPos = targetPosition;
            float y = position.y;
            targetPos.y = y;

            float currentRemain = math.distance(position, targetPos);
            float3 steering = float3.zero;
            steering += Seek(agent.ValueRO, position, targetPos, deltaTime);
            steering += CollisionAvoidance(obstacles, agent.ValueRO, position, agent.ValueRO.velocity);

            steering = math.normalize(steering) * agent.ValueRO.maxForce;
            steering = steering / agent.ValueRO.mass;
            steering.y = 0.0f;

            agent.ValueRW.velocity = math.normalize(agent.ValueRO.velocity + steering) * agent.ValueRO.maxSpeed * deltaTime;
            agent.ValueRW.velocity.y = 0;

            position = position + agent.ValueRO.velocity;
            position.y = y;

            float3 movDir = math.normalize(agent.ValueRO.velocity);
            movDir.y = 0;
            quaternion moveDirRot = quaternion.Euler(movDir);

            position += CollisionRaycast(agent.ValueRO, position, moveDirRot, movDir) * deltaTime;
            agent.ValueRW.lookDirection = math.normalize(position - transform.ValueRO.Position);

            float predicMoveDistance = math.distance(transform.ValueRO.Position, position);
            float predicRemainDistance = math.distance(position, targetPos);
            
            if (predicRemainDistance > (agent.ValueRO.radius + 0.1f) && predicMoveDistance < currentRemain)
                transform.ValueRW.Position = position;
            else
                agent.ValueRW.arrival = true;
        
            agentObstacle.ValueRW.position = transform.ValueRO.Position;
        }

        private void UpdateRoatation(ref AgentAspect agentAsepct, float deltaTime)
        {
            Agent agent = agentAsepct.agent.ValueRO;
            float3 forward = math.lerp(agentAsepct.localTransform.ValueRO.Forward(), agent.lookDirection, deltaTime * agent.rotLerpSpeed);
            agentAsepct.localTransform.ValueRW.Rotation = quaternion.LookRotation(forward, new float3(0, 1, 0));
        }

        private bool IsArrival(RefRW<Agent> agent)
        {
            if (agent.ValueRO.arrival == false)
                return false;

            if (isDirty && agent.ValueRO.arrival)
            {
                agent.ValueRW.arrival = false;
                isDirty = false;
            }
            return agent.ValueRO.arrival;
        }

        public void Execute(AgentAspect agentAsepct)
        {
            if (IsArrival(agentAsepct.agent))
                return;

            UpdateMove(obstacles, ref agentAsepct, targetPosition, deltaTime);
            UpdateRoatation(ref agentAsepct, deltaTime);
        }
    }
}
