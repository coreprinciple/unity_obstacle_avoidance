using UnityEngine;
using System.Collections.Generic;

namespace avoidance.mono
{
    public static class AgentUtil
    {
        private static bool LineIntersectsCircle(Vector3 ahead, Vector3 ahead2, Vector3 point, float radius)
        {
            return Vector3.Distance(point, ahead) <= radius || Vector3.Distance(point, ahead2) <= radius;
        }

        public static void CheckNearObstacles(List<Transform> obstacles, Transform transform, float rayRadius, int obstacleLayer)
        {
            RaycastHit[] hits = Physics.SphereCastAll(transform.position, rayRadius, Vector3.up, rayRadius, 1 << obstacleLayer);
            obstacles.Clear();

            foreach (var hit in hits)
                obstacles.Add(hit.transform);
        }

        public static Transform FindMostThreateningObstacle(Transform agentTransform, Vector3 position, Vector3 ahead, Vector3 ahead2, List<Transform> obstacles, float radius)
        {
            Transform mostThreatening = null;

            foreach (var obstacle in obstacles)
            {
                if (obstacle == agentTransform)
                    continue;

                Vector3 obstaclePosition = obstacle.transform.position;

                if (LineIntersectsCircle(ahead, ahead2, obstaclePosition, radius) == false)
                    continue;

                if (mostThreatening == null || Vector3.Distance(position, obstaclePosition) < Vector3.Distance(position, mostThreatening.transform.position))
                    mostThreatening = obstacle;
            }
            return mostThreatening;
        }
    }
}
