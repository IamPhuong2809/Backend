#ifndef NAV2_IPSO_PLANNER__TRAJECTORY_GENERATION_HPP_
#define NAV2_IPSO_PLANNER__TRAJECTORY_GENERATION_HPP_

#include <vector>
#include <utility>
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_ipso_planner
{
   class TrajectoryGeneration
   {
    public:
        TrajectoryGeneration();
        ~TrajectoryGeneration();

        static int splineOrder; //Set the order of the B-spline curve

        /**
         * @brief Using quasi uniform B-spline curves for trajectory generation
         * @param plan    Key points&generated trajectories
         * @param k       Order of B-spline curve
         */
        void B_spline_curve(std::vector<std::pair<double, double>> &plan, int k, const unsigned char* global_costmap, int nx, int ny);


        /**
         * @brief Generate B-spline curve control points
         * @param start_d           starting point
         * @param goal_d            Target point
         * @param initial_position  Intermediate control point
         * @param initial_point     B-spline control points
         */
        void GenerateControlPoints(const std::pair<double, double>& start_d, const std::pair<double, double>& goal_d,
                            const std::vector<std::pair<int, int>>& initial_position,
                            std::vector<std::pair<double, double>>& initial_point);

        /**
         * @brief Calculate the distance between two points
         * @param point1     First point
         * @param point2     Second point
         * @return the distance between two points
         */
        double calculateDistance(const std::pair<double, double>& point1, const std::pair<double, double>& point2);

        /**
         * @brief  Calculate path length
         * @param path  The path to be calculated
         * @return path length
         */
        double calculatePathLength(const std::vector<std::pair<double, double>>& path);

        std::pair<double, double> findNearestSafePoint(int x, int y, const unsigned char* global_costmap, int nx, int ny, unsigned char lethal_cost);


    private:
        double BaseFun(int i, int k, double u, std::vector<double> NodeVector);
        const unsigned char* global_costmap_;
        int nx_;
        int ny_;
};

}

#endif // NAV2_IPSO_PLANNER__TRAJECTORY_GENERATION_HPP_