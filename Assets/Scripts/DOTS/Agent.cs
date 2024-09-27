using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;

namespace avoidance.dots
{
    [BurstCompile]
    public struct Agent : IComponentData
    {
        public bool arrival;

        public int id;

        public float maxAvoidForce;
        public float maxSeeAhead;
        public float maxVelocity;
        public float maxSpeed;
        public float maxForce;
        public float mass;
        public float radius;

        public float rotLerpSpeed;

        public float3 targetPosition;
        public float3 forward;
        public float3 velocity;
        public float3 targetPos;
        public float3 lookDirection;
    }
}

