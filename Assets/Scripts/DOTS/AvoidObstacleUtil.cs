namespace avoidance.dots
{
    public class AvoidObstacleUtil
    {
        protected static AvoidObstacleUtil sInstance;
        private int _obstacleID;

        public static AvoidObstacleUtil Instance()
        {
            if (sInstance == null)
                sInstance = new AvoidObstacleUtil();
            return sInstance;
        }

        public int GetNewObstacleID() => ++_obstacleID;
    }
}

