#ifndef NAV2_IPSO_PLANNER__IPSO_HPP_
#define NAV2_IPSO_PLANNER__IPSO_HPP_

#pragma once

#include <random>
#include <thread>
#include <mutex>
#include <vector>
#include <memory>

#include "nav2_ipso_planner/TrajectoryGeneration.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_costmap_2d/cost_values.hpp" 

using PositionSequence = std::vector<std::vector<std::pair<int, int>>>;

namespace nav2_ipso_planner
{
  struct Particle
  {
    std::vector<std::pair<int, int>> position;               // Particle position
    std::vector<std::pair<int, int>> velocity;               // Particle velocity
    double fitness;                                          // Particle fitness
    std::vector<std::pair<int, int>> personal_best_pos;      // Personal best position in iteration
    double personal_best_fitness;                            // Personal best fitness in iteration

    Particle() = default;  

    Particle(const std::vector<std::pair<int, int>>& initial_position,
             const  std::vector<std::pair<int, int>>& initial_velocity,
            double initial_fitness)
        : position(initial_position),
          velocity(initial_velocity),
          fitness(initial_fitness),
          personal_best_pos(initial_position),
          personal_best_fitness(initial_fitness)
    {
    }
  };

  /**
   * @brief Class for objects that plan using the PSO algorithm
   */

  class IPSO 
  {
  public:
    /**
     * @brief Construct a new PSO object
     * @param nx            	  pixel number in costmap x direction
     * @param ny            	  pixel number in costmap y direction
     * @param resolution    	  costmap resolution
     * @param origin_x      	  origin coordinate in the x direction.
     * @param origin_y      	  origin coordinate in the y direction.
     * @param n_particles	  number of particles
     * @param n_inherited   	  number of inherited particles
     * @param pointNum      	  number of position points contained in each particle
     * @param w_inertial	  inertia weight
     * @param w_social		  social weight
     * @param w_cognitive	  cognitive weight
     * @param obs_factor    	  obstacle factor(greater means obstacles)
     * @param max_speed		  The maximum movement speed of particles
     * @param initposmode	  Set the generation mode for the initial position points of the particle swarm
     * @param pub_particles 	  Boolean flag to publish particles.
     * @param ipso_max_iter	  maximum iterations
     */
    IPSO(
   int nx, int ny, double resolution, double origin_x, double origin_y,
   int n_particles, int n_inherited, int pointNum, double w_inertial_min, double w_inertial_max,
   double w_social_lower, double w_social_upper, double w_cognitive_lower, double w_cognitive_upper, 
   double obs_factor, int max_speed, int initposmode, bool pub_particles, int ipso_max_iter,
   const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

    ~IPSO();

    /**
     * @brief IPSO implementation
     * @param global_costmap global costmap
     * @param start         start node
     * @param goal          goal node
     * @param path          optimal path consists of Node
     * @return  true if path found, else false
     */
    bool plan(const unsigned char* global_costmap,
             const std::pair<int, int>& start,
             const std::pair<int, int>& goal,
             std::vector<std::pair<int, int>>& path);

    /**
     * @brief Generate n particles with pointNum_ positions each within the map range
     * @param initialPositions The initial position sequence of particle swarm
     * @param start_d        starting point
     * @param goal_d         Target point
     */
    void generateRandomInitialPositions(PositionSequence &initialPositions,
                                        const std::pair<double, double> start_d,
                                        const std::pair<double, double> goal_d);

    /**
     * @brief Generate an initial position point sequence within a circular area, located within the map range
     * @param initialPositions The initial position sequence of particle swarm
     * @param start_d        starting point
     * @param goal_d         Target point
     */
    void generateCircularInitialPositions(PositionSequence &initialPositions,
                                          const std::pair<double, double> start_d,
                                          const std::pair<double, double> goal_d);

    /**
     * @brief Calculate Obstacle avoidance cost
     * @param global_costmap   global costmap
     * @param pso_path         Path to perform collision detection
     * @return  The collision cost of the path
     */
    double ObstacleCost(const unsigned char* global_costmap,
                        const std::vector<std::pair<double, 
                        double>>& pso_path);

    /**
     * @brief A function to update the particle velocity
     * @param particle     Particles to be updated for velocity
     * @param global_best  Global optimal particle
     * @param gen          randomizer
     */
    void updateParticleVelocity(Particle& particle,
                                const Particle& global_best,
                                std::mt19937& gen); 

    /**
     * @brief A function to update the particle position
     * @param particle     Particles to be updated for velocity
     */
    void updateParticlePosition(Particle& particle);

    
    /**
     * @brief Particle update optimization iteration
     * @param particle       Particles to be updated for velocity
     * @param best_particle  Global optimal particle
     * @param start_d        starting point
     * @param goal_d         Target point
     * @param index_i        Particle ID
     * @param global_costmap global costmap
     * @param gen            randomizer
     */
    void optimizeParticle(Particle& particle, Particle& best_particle, 
                          const unsigned char* global_costmap, 
                          const std::pair<double, double>& start_d, 
                          const std::pair<double, double>& goal_d,
                          const int& index_i,std::mt19937& gen);
    
    /**
     * @brief Clamps a value within a specified range.
     * @tparam T             The type of the values to be clamped.
     * @param value          The value to be clamped.
     * @param low            The lower bound of the range.
     * @param high           The upper bound of the range.
     * @return const T&      The clamped value within the specified range.
     */
    template <typename T>
    const T& clamp(const T& value, const T& low, const T& high) 
    {
     return std::max(low, std::min(value, high));
    }

    /**
     * @brief Custom comparison function for ascending order.
     * @param a   	The first element to be compared.
     * @param b   	The second element to be compared.
     * @return bool  	True if 'a' is less than 'b', indicating ascending order; otherwise, false.
     */

    /**
     * @brief Custom comparison function for descending order.
     * @param a   	The first element to be compared.
     * @param b   	The second element to be compared.
     * @return bool  	True if 'a' is greater than 'b', indicating descending order; otherwise, false.
     */


    /**
     * @brief Publishes   particle markers based on given positions and index.
     * @param positions   Vector of pairs representing positions (x, y) of particles.
     * @param index       Index identifier for the particles.
     */
    void publishParticleMarkers(const std::vector<std::pair<int, int>>& positions, 
                                const int& index);

    /**
     * @brief Transform from grid map(x, y) to grid index(i)
     * @param x grid map x
     * @param y grid map y
     * @return  index
     */

    int grid2Index (int x, int y);

    bool isLineCollisionFree(const std::pair<double, double>& p1, const std::pair<double, double>& p2, const unsigned char* global_costmap);


  protected:
    const unsigned char* global_costmap_;
    int nx_,ny_,ns_;              			// the number of grids on the x-axis and y-axis of the map, as well as the total number of grids   
    double resolution_;           			// map resolution
    double origin_x_,origin_y_;   			// origin coordinate in the x、y direction.
    int n_particles_;             			// number of particles
    int n_inherited_;             			// number of inherited particles
    int pointNum_;                			// number of position points contained in each particle
    // Weight coefficients for fitness calculation: inertia weight, social weight, cognitive weight
    double w_inertial_min_, w_inertial_max_; 
    double w_social_lower_, w_social_upper_; 
    double w_cognitive_lower_, w_cognitive_upper_;   
    double obs_factor_;           			// obstacle factor(greater means obstacles)
    int max_speed_;               			// The maximum velocity of particle motion
    int initposmode_;             			// Set the generation mode for the initial position points of the particle swarm
    bool pub_particles_;          			// boolean flag to publish particles.
    int ipso_max_iter_;            			// maximum iterations
    

  private:
    unsigned char lethal_cost_ = nav2_costmap_2d::LETHAL_OBSTACLE;                 //lethal cost
    unsigned char inflated_cost_ = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr particle_pub_;                    //The publisher of real-time particle visualization 
    int GlobalBest_particle_;                       //The ID of the globally optimal particle
    //int inflationRadiusCells_;
    //double sigma_;   
    double w_social_;
    double w_cognitive_;
    double Favg_;
    double FGbest_;
    std::mutex particles_lock_;                     //thread lock
    std::vector<Particle> inherited_particles_;     //inherited particles
    TrajectoryGeneration path_generation;           //Path generation
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                  // Create Publisher/log from ros2 node
    static bool ascendingOrder(int a, int b) { return a < b; }
    static bool descendingOrder(int a, int b) { return a > b; }
  };

}  // namespace nav2_ipso_planner

#endif //NAV2_IPSO_PLANNER__IPSO_HPP_
