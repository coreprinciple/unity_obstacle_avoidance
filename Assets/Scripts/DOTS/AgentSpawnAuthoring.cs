using UnityEngine;
using Unity.Entities;

namespace avoidance.dots
{
    public class AgentSpawnAuthoring : MonoBehaviour
    {
        public GameObject prefab;

        public class AgentBaker : Baker<AgentSpawnAuthoring>
        {
            public override void Bake(AgentSpawnAuthoring authoring)
            {
                if (authoring.prefab == null)
                    return;

                var entity = GetEntity(TransformUsageFlags.None);
                AddComponent(entity, new AgentSpawn { Prefab = GetEntity(authoring.prefab, TransformUsageFlags.Dynamic) });
            }
        }
    }

    public struct AgentSpawn : IComponentData
    {
        public Entity Prefab;
    }
}
