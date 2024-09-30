using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace avoidance.dots
{
    public class ObstacleAuthoring : MonoBehaviour
    {
        public GameObject prefab;

        public class ObstacleBaker : Baker<ObstacleAuthoring>
        {
            public override void Bake(ObstacleAuthoring authoring)
            {
                if (authoring.prefab == null)
                    return;

                Entity entity = GetEntity(authoring.prefab, TransformUsageFlags.Dynamic);
                AddComponent(entity, new Obstacle() { position = authoring.prefab.transform.position });
            }
        }
    }

    public struct Obstacle : IComponentData
    {
        public int id;
        public float3 position;
        public bool isAgent;
    }
}

