using Unity.Physics;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;

namespace avoidance.dots
{
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

        private void UpdateMove(NativeList<Obstacle> obstacles, ref Agent agent, ref Obstacle obstacle, ref LocalTransform transform, float3 targetPosition, float deltaTime)
        {
            float3 position = transform.Position;
            float3 targetPos = targetPosition;
            float y = position.y;
            targetPos.y = y;

            float currentRemain = math.distance(position, targetPos);
            float3 steering = float3.zero;
            steering += Seek(agent, position, targetPos, deltaTime);
            steering += CollisionAvoidance(obstacles, agent, position, agent.velocity);

            steering = math.normalize(steering) * agent.maxForce;
            steering = steering / agent.mass;
            steering.y = 0.0f;

            agent.velocity = math.normalize(agent.velocity + steering) * agent.maxSpeed * deltaTime;
            agent.velocity.y = 0;

            position = position + agent.velocity;
            position.y = y;

            float3 movDir = math.normalize(agent.velocity);
            movDir.y = 0;
            quaternion moveDirRot = quaternion.Euler(movDir);

            position += CollisionRaycast(agent, position, moveDirRot, movDir) * deltaTime;
            agent.lookDirection = math.normalize(position - transform.Position);

            float predicMoveDistance = math.distance(transform.Position, position);
            float predicRemainDistance = math.distance(position, targetPos);

            if (predicRemainDistance > agent.radius && predicMoveDistance < currentRemain)
                transform.Position = position;
            else
                agent.arrival = true;

            obstacle.position = transform.Position;
        }

        private void UpdateRoatation(Agent agent, ref LocalTransform transform, float deltaTime)
        {
            float3 forward = math.lerp(transform.Forward(), agent.lookDirection, deltaTime * agent.rotLerpSpeed);
            transform.Rotation = quaternion.LookRotation(forward, new float3(0, 1, 0));
        }

        private bool IsArrival(ref Agent agent)
        {
            if (agent.arrival == false)
                return false;

            if (isDirty && agent.arrival)
            {
                agent.arrival = false;
                isDirty = false;
            }
            return agent.arrival;
        }

        public void Execute(ref Agent agent, ref Obstacle obstacle, ref LocalTransform transform)
        {
            if (IsArrival(ref agent))
                return;

            UpdateMove(obstacles, ref agent, ref obstacle, ref transform, targetPosition, deltaTime);
            UpdateRoatation(agent, ref transform, deltaTime);
        }
    }
}
