using Unity.Entities;
using Unity.Transforms;

namespace avoidance.dots
{
    public readonly partial struct AgentAspect : IAspect
    {
        public readonly Entity self;

        public readonly RefRW<Agent> agent;
        public readonly RefRW<Obstacle> obstacle;
        public readonly RefRW<LocalTransform> localTransform;
    }
}

